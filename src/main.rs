extern crate rtt;
extern crate rand;
extern crate gfx_core;
extern crate env_logger;
extern crate piston_window;
#[macro_use] extern crate log;
#[macro_use] extern crate clap;

use std::{io, thread, process};
use std::sync::mpsc;
use std::path::PathBuf;

use clap::Arg;
use piston_window::{
    OpenGL,
    PistonWindow,
    WindowSettings,
    TextureSettings,
    Viewport,
    Glyphs,
    Event,
    Input,
    Button,
    ButtonArgs,
    Motion,
    Key,
};

mod common;
mod rtt_slave;

use common::{
    CircleArea,
    Field,
    FieldConfig,
    MasterReq,
    SlaveRep,
};

fn main() {
    env_logger::init();
    match run() {
        Ok(()) =>
            info!("graceful shutdown"),
        Err(e) => {
            error!("fatal error: {:?}", e);
            process::exit(1);
        },
    }
}

#[derive(Debug)]
enum Error {
    MissingParameter(&'static str),
    Piston(PistonError),
    ThreadSpawn(io::Error),
    ThreadJoin(Box<std::any::Any + Send + 'static>),
}

#[derive(Debug)]
enum PistonError {
    BuildWindow(String),
    LoadFont { file: String, error: io::Error, },
    DrawText(gfx_core::factory::CombinedError),
}

const CONSOLE_HEIGHT: u32 = 32;
const BORDER_WIDTH: u32 = 16;
const SCREEN_WIDTH: u32 = 640;
const SCREEN_HEIGHT: u32 = 480;

fn run() -> Result<(), Error> {
    let matches = app_from_crate!()
        .arg(Arg::with_name("assets-dir")
             .short("a")
             .long("assets-dir")
             .value_name("DIR")
             .help("Graphics resources directory")
             .default_value("./assets")
             .takes_value(true))
        .get_matches();

    let assets_dir = matches.value_of("assets-dir")
        .ok_or(Error::MissingParameter("assets-dir"))?;

    let opengl = OpenGL::V4_1;
    let mut window: PistonWindow = WindowSettings::new("RTT demo", [SCREEN_WIDTH, SCREEN_HEIGHT])
        .exit_on_esc(true)
        .opengl(opengl)
        .build()
        .map_err(PistonError::BuildWindow)
        .map_err(Error::Piston)?;

    let mut font_path = PathBuf::from(assets_dir);
    font_path.push("FiraSans-Regular.ttf");
    let mut glyphs = Glyphs::new(&font_path, window.factory.clone(), TextureSettings::new())
        .map_err(|e| Error::Piston(PistonError::LoadFont {
            file: font_path.to_string_lossy().to_string(),
            error: e,
        }))?;

    let field_config = FieldConfig::new(
        SCREEN_WIDTH as f64,
        SCREEN_HEIGHT as f64,
    );
    let mut field = Field::generate(field_config);
    let mut cursor = None;

    let (master_tx, slave_rx) = mpsc::channel();
    let (slave_tx, master_rx) = mpsc::channel();

    let slave = thread::Builder::new()
        .name("RTT demo slave".to_string())
        .spawn(move || rtt_slave::run(slave_rx, slave_tx))
        .map_err(Error::ThreadSpawn)?;

    while let Some(event) = window.next() {
        let maybe_result = window.draw_2d(&event, |context, g2d| {
            use piston_window::{clear, text, ellipse, Transformed};
            clear([0.0, 0.0, 0.0, 1.0], g2d);

            // draw menu
            text::Text::new_color([0.0, 1.0, 0.0, 1.0], 16).draw(
                &format!("Mode: [ path ]; press <M> to switch mode, <C> to clear or <Q> to exit"),
                &mut glyphs,
                &context.draw_state,
                context.transform.trans(5.0, 20.0),
                g2d
            ).map_err(PistonError::DrawText)?;

            if let Some(tr) = ViewportTranslator::new(&context.viewport) {
                // draw start
                ellipse(
                    [0.75, 0.75, 0.0, 1.0],
                    [
                        tr.x(field.config.start_area.center.x) - field.config.start_area.radius,
                        tr.y(field.config.start_area.center.y) - field.config.start_area.radius,
                        field.config.start_area.radius * 2.,
                        field.config.start_area.radius * 2.,
                    ],
                    context.transform,
                    g2d,
                );
                // draw finish
                ellipse(
                    [0.2, 0.2, 1.0, 1.0],
                    [
                        tr.x(field.config.finish_area.center.x) - field.config.finish_area.radius,
                        tr.y(field.config.finish_area.center.y) - field.config.finish_area.radius,
                        field.config.finish_area.radius * 2.,
                        field.config.finish_area.radius * 2.,
                    ],
                    context.transform,
                    g2d,
                );
                // draw obstacles
                for obstacle in field.obstacles.iter() {
                    ellipse(
                        [0.5, 1.0, 0.5, 1.0],
                        [
                            tr.x(obstacle.center.x) - obstacle.radius,
                            tr.y(obstacle.center.y) - obstacle.radius,
                            obstacle.radius * 2.,
                            obstacle.radius * 2.,
                        ],
                        context.transform,
                        g2d,
                    );
                }
                // draw cursor
                if let Some((cx, cy)) = cursor {
                    ellipse(
                        [0., 1.0, 0., 1.0],
                        [cx - 5., cy - 5., 10., 10.,],
                        context.transform,
                        g2d,
                    );
                }
            }

            Ok(())
        });
        if let Some(result) = maybe_result {
            let () = result.map_err(Error::Piston)?;
        }

        match event {
            Event::Input(Input::Button(ButtonArgs { button: Button::Keyboard(Key::Q), .. })) =>
                break,
            Event::Input(Input::Move(Motion::MouseCursor(x, y))) =>
                cursor = Some((x, y)),
            Event::Input(Input::Cursor(false)) =>
                cursor = None,
            Event::Input(Input::Cursor(true)) =>
                cursor = Some((0., 0.)),
            _ =>
                (),
        }
    }

    let _ = master_tx.send(MasterReq::Terminate);
    let () = slave.join().map_err(Error::ThreadJoin)?;

    Ok(())
}

struct ViewportTranslator {
    scale_x: f64,
    scale_y: f64,
    min_x: f64,
    min_y: f64,
}

impl ViewportTranslator {
    fn new(viewport: &Option<Viewport>) -> Option<ViewportTranslator> {
        let (w, h) = viewport
            .map(|v| (v.draw_size[0], v.draw_size[1]))
            .unwrap_or((SCREEN_WIDTH, SCREEN_HEIGHT));

        if (w <= 2 * BORDER_WIDTH) || (h <= BORDER_WIDTH + CONSOLE_HEIGHT) {
            None
        } else {
            let bounds = (0., 0., SCREEN_WIDTH as f64, SCREEN_HEIGHT as f64);
            Some(ViewportTranslator {
                scale_x: (w - BORDER_WIDTH - BORDER_WIDTH) as f64 / (bounds.2 - bounds.0),
                scale_y: (h - BORDER_WIDTH - CONSOLE_HEIGHT) as f64 / (bounds.3 - bounds.1),
                min_x: bounds.0,
                min_y: bounds.1,
            })
        }
    }

    fn x(&self, x: f64) -> f64 {
        (x - self.min_x) * self.scale_x + BORDER_WIDTH as f64
    }

    fn y(&self, y: f64) -> f64 {
        (y - self.min_y) * self.scale_y + CONSOLE_HEIGHT as f64
    }
}
