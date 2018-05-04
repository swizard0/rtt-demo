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
    Glyphs,
    Event,
    Input,
    Button,
    ButtonArgs,
    ButtonState,
    MouseButton,
    Motion,
    Key,
};

mod common;
mod rtt_slave;

use common::{
    Point,
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

    let (master_tx, slave_rx) = mpsc::channel();
    let (slave_tx, master_rx) = mpsc::channel();

    let slave = thread::Builder::new()
        .name("RTT demo slave".to_string())
        .spawn(move || rtt_slave::run(slave_rx, slave_tx))
        .map_err(Error::ThreadSpawn)?;

    let mut env = Env::new();
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

            if let Some((_width, _height)) = context.viewport.map(|v| (v.draw_size[0], v.draw_size[1])) {
                // draw start
                ellipse(
                    [0.75, 0.75, 0.0, 1.0],
                    [
                        env.field.config.start_area.center.x - env.field.config.start_area.radius,
                        env.field.config.start_area.center.y - env.field.config.start_area.radius,
                        env.field.config.start_area.radius * 2.,
                        env.field.config.start_area.radius * 2.,
                    ],
                    context.transform,
                    g2d,
                );
                // draw finish
                ellipse(
                    [0.2, 0.2, 1.0, 1.0],
                    [
                        env.field.config.finish_area.center.x - env.field.config.finish_area.radius,
                        env.field.config.finish_area.center.y - env.field.config.finish_area.radius,
                        env.field.config.finish_area.radius * 2.,
                        env.field.config.finish_area.radius * 2.,
                    ],
                    context.transform,
                    g2d,
                );
                // draw obstacles
                for obstacle in env.field.obstacles.iter() {
                    ellipse(
                        [0.5, 1.0, 0.5, 1.0],
                        [
                            obstacle.center.x - obstacle.radius,
                            obstacle.center.y - obstacle.radius,
                            obstacle.radius * 2.,
                            obstacle.radius * 2.,
                        ],
                        context.transform,
                        g2d,
                    );
                }
                // draw cursor
                if let Some((mx, my)) = env.cursor {
                    if let Some((cx, cy)) = env.obs_center {
                        let radius = coords_radius(cx, cy, mx, my);
                        ellipse(
                            [1.0, 0., 0., 1.0],
                            [cx - radius, cy - radius, radius * 2., radius * 2.,],
                            context.transform,
                            g2d,
                        );
                    } else {
                        ellipse(
                            [1.0, 0., 0., 1.0],
                            [mx - 5., my - 5., 10., 10.,],
                            context.transform,
                            g2d,
                        );
                    }
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
                env.set_cursor(x, y),
            Event::Input(Input::Cursor(false)) =>
                env.reset_cursor(),
            Event::Input(Input::Button(ButtonArgs { button: Button::Mouse(MouseButton::Left), state: ButtonState::Release, .. })) =>
                env.toggle_obs(),
            Event::Input(Input::Resize(width, height)) =>
                env.reset(width, height),
            _ =>
                (),
        }
    }

    let _ = master_tx.send(MasterReq::Terminate);
    let () = slave.join().map_err(Error::ThreadJoin)?;

    Ok(())
}

struct Env {
    field: Field,
    cursor: Option<(f64, f64)>,
    obs_center: Option<(f64, f64)>,
}

impl Env {
    fn new() -> Env {
        Env {
            field: Field::generate(FieldConfig::new(
                0.,
                CONSOLE_HEIGHT as f64,
                SCREEN_WIDTH as f64,
                SCREEN_HEIGHT as f64,
            )),
            cursor: None,
            obs_center: None,
        }
    }

    fn reset(&mut self, width: u32, height: u32) {
        self.field = Field::generate(FieldConfig::new(
            0.,
            CONSOLE_HEIGHT as f64,
            width as f64,
            height as f64,
        ));
        self.cursor = None;
        self.obs_center = None;
    }

    fn set_cursor(&mut self, x: f64, y: f64) {
        self.cursor = if y < CONSOLE_HEIGHT as f64 {
            None
        } else {
            Some((x, y))
        }
    }

    fn reset_cursor(&mut self) {
        self.cursor = None;
        self.obs_center = None;
    }

    fn toggle_obs(&mut self) {
        if let Some((mx, my)) = self.cursor {
            self.obs_center = if let Some((cx, cy)) = self.obs_center {
                self.field.obstacles.push(CircleArea {
                    center: Point { x: cx, y: cy, },
                    radius: coords_radius(cx, cy, mx, my),
                });
                None
            } else {
                Some((mx, my))
            };
        }
    }
}

fn coords_radius(xa: f64, ya: f64, xb: f64, yb: f64) -> f64 {
    ((xb - xa) * (xb - xa) + (yb - ya) * (yb - ya)).sqrt()
}
