use rand::{thread_rng, Rng};

#[derive(Clone, Copy, Debug)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

impl Point {
    pub fn sq_dist(&self, other: &Point) -> f64 {
        ((self.x - other.x) * (self.x - other.x)) + ((self.y - other.y) * (self.y - other.y))
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CircleArea {
    pub center: Point,
    pub radius: f64,
}

#[derive(Clone, Debug)]
pub struct FieldConfig {
    pub start_area: CircleArea,
    pub finish_area: CircleArea,
    pub field_area: (f64, f64, f64, f64),
}

impl FieldConfig {
    pub fn new(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> FieldConfig {
        let width = max_x - min_x;
        let height = max_y - min_y;
        let area_side = if width < height { width } else { height };
        let area_side = if area_side < 40. { area_side } else { 40. };
        let diameter = area_side / 2.;
        let radius = diameter / 2.;
        let padding2 = diameter;
        let padding = padding2 / 2.;

        FieldConfig {
            start_area: CircleArea {
                center: Point {
                    x: min_x + padding + radius,
                    y: min_y + padding + radius,
                },
                radius,
            },
            finish_area: CircleArea {
                center: Point {
                    x: max_x - padding - radius,
                    y: max_y - padding - radius,
                },
                radius,
            },
            field_area: (min_x, min_y, max_x, max_y),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Field {
    pub config: FieldConfig,
    pub start: Point,
    pub obstacles: Vec<CircleArea>,
}

impl Field {
    pub fn generate(config: FieldConfig) -> Field {
        let mut rng = thread_rng();
        let rnd_radius = rng.gen_range(0., config.start_area.radius);
        let rnd_angle = rng.gen_range(0., ::std::f64::consts::PI * 2.);
        let start = Point {
            x: config.start_area.center.x + rnd_radius * rnd_angle.cos(),
            y: config.start_area.center.y + rnd_radius * rnd_angle.sin(),
        };
        Field { config, start, obstacles: Vec::new(), }
    }
}

pub enum MasterPacket {
    Solve(Field),
    Abort,
    Terminate,
}

pub enum SlavePacket {
    RouteDone(Vec<Point>),
}
