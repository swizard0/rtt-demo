use std::sync::mpsc;

use rtt::{self, util::{no_err, rtt::vec_slist::{EmptyRandomTree, RandomTree, NodeRef}}};
use rand::{self, Rng};

use super::common::{
    MasterPacket,
    SlavePacket,
    Field,
    Point,
};

pub fn run(rx: mpsc::Receiver<MasterPacket>, tx: mpsc::Sender<SlavePacket>) {
    run_idle(&rx, &tx);
}

fn run_idle(rx: &mpsc::Receiver<MasterPacket>, tx: &mpsc::Sender<SlavePacket>) {
    loop {
        match rx.recv() {
            Ok(MasterPacket::Solve(field)) =>
                if run_solve(rx, tx, field) {
                    break;
                },
            Ok(MasterPacket::Terminate) =>
                break,
            Ok(MasterPacket::Abort) =>
                (),
            Err(mpsc::RecvError) =>
                break,
        }
    }
}

struct Trans {
    field: Field,
}

impl Trans {
    fn new(field: Field) -> Trans {
        Trans { field, }
    }

    fn goal_reached(&self, point: &Point) -> bool {
        self.field.config.finish_area.center.sq_dist(point) <
            self.field.config.finish_area.radius * self.field.config.finish_area.radius
    }

    fn trans_root(&mut self, empty_rtt: EmptyRandomTree<Point>) -> Result<RttNodeFocus, !> {
        let rtt = empty_rtt.add_root(self.field.start);
        let root_ref = rtt.root();
        no_err(RttNodeFocus {
            rtt,
            node_ref: root_ref,
            goal_reached: self.goal_reached(&self.field.start),
        })
    }

    fn has_route(&self, &(ref rtt, ref node_ref): &(RandomTree<Point>, NodeRef), dst: &Point) -> bool {
        let src = rtt.get_state(node_ref);
        if src.sq_dist(dst) <= 0. {
            return false;
        }
        let seg_v = Point { x: dst.x - src.x, y: dst.y - src.y, };
        let seg_v_len = (seg_v.x * seg_v.x + seg_v.y * seg_v.y).sqrt();

        for obstacle in self.field.obstacles.iter() {
            let closest_point = {
                let pt_v = Point { x: obstacle.center.x - src.x, y: obstacle.center.y - src.y, };
                let seg_v_unit = Point { x: seg_v.x / seg_v_len, y: seg_v.y / seg_v_len, };
                let proj = pt_v.x * seg_v_unit.x + pt_v.y * seg_v_unit.y;
                if proj <= 0. {
                    src.clone()
                } else if proj >= seg_v_len {
                    dst.clone()
                } else {
                    let proj_v = Point { x: seg_v_unit.x * proj, y: seg_v_unit.y * proj, };
                    Point { x: proj_v.x + src.x, y: proj_v.y + src.y, }
                }
            };
            if closest_point.sq_dist(&obstacle.center) < obstacle.radius * obstacle.radius {
                return false;
            }
        }

        return true;
    }
}

struct RttNodeFocus {
    rtt: RandomTree<Point>,
    node_ref: NodeRef,
    goal_reached: bool,
}

impl RttNodeFocus {
    fn into_direct_path(self) -> Result<Vec<Point>, !> {
        let mut rev_path: Vec<_> = self.rtt.into_path(self.node_ref).collect();
        rev_path.reverse();
        no_err(rev_path)
    }

    fn into_rtt(self) -> Result<RandomTree<Point>, !> {
        no_err(self.rtt)
    }
}

fn run_solve(rx: &mpsc::Receiver<MasterPacket>, tx: &mpsc::Sender<SlavePacket>, field: Field) -> bool {
    let mut rng = rand::thread_rng();

    let planner = rtt::Planner::new(EmptyRandomTree::new());

    let mut trans = Trans::new(field);
    let mut planner_node = planner.add_root(|empty_rtt| trans.trans_root(empty_rtt)).unwrap();

    loop {
        if planner_node.rtt_node().goal_reached {
            let path = planner_node.into_path(RttNodeFocus::into_direct_path).unwrap();
            tx.send(SlavePacket::RouteDone(path)).ok();
            return false;
        }

        let mut planner_sample = planner_node.prepare_sample(RttNodeFocus::into_rtt).unwrap();
        loop {
            match rx.try_recv() {
                Ok(MasterPacket::Solve(..)) =>
                    (),
                Ok(MasterPacket::Terminate) =>
                    return true,
                Ok(MasterPacket::Abort) =>
                    return false,
                Err(mpsc::TryRecvError::Empty) =>
                    (),
                Err(mpsc::TryRecvError::Disconnected) =>
                    return true,
            }

            let sample = Point {
                x: rng.gen_range(trans.field.config.field_area.0, trans.field.config.field_area.2),
                y: rng.gen_range(trans.field.config.field_area.1, trans.field.config.field_area.3),
            };
            let planner_pick = planner_sample.sample(no_err).unwrap();
            let planner_closest = planner_pick.nearest_node(|rtt: RandomTree<Point>| {
                let mut closest;
                {
                    let points = rtt.states();
                    closest = (points.root.0, sample.sq_dist(points.root.1));
                    for (node_ref, point) in points.children {
                        let sq_dist = sample.sq_dist(point);
                        if sq_dist < closest.1 {
                            closest = (node_ref, sq_dist);
                        }
                    }
                }
                no_err((rtt, closest.0))
            }).unwrap();

            if trans.has_route(planner_closest.rtts_node(), &sample) {
                planner_node = planner_closest.transition(|(rtt, node_ref): (RandomTree<_>, _)| {
                    let goal_reached = trans.goal_reached(rtt.get_state(&node_ref));
                    no_err(RttNodeFocus { rtt, node_ref, goal_reached, })
                }).unwrap();
                break;
            } else {
                planner_sample = planner_closest.no_transition(|(rtt, _)| no_err(rtt)).unwrap();
            }
        }
    }
}
