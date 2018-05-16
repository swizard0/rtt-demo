use std::sync::mpsc;
use std::collections::HashSet;

use rtt::{self, util::{NeverError, rtt::vec_slist::{EmptyRandomTree, RandomTree, NodeRef}}};
use rand::{self, Rng};

use super::common::{
    MasterPacket,
    SlavePacket,
    Field,
    Point,
    DebugImage,
    SampleTry,
};

pub fn run(rx: mpsc::Receiver<MasterPacket>, tx: mpsc::Sender<SlavePacket>) {
    run_idle(&rx, &tx);
}

fn run_idle(rx: &mpsc::Receiver<MasterPacket>, tx: &mpsc::Sender<SlavePacket>) {
    loop {
        match rx.recv() {
            Ok(MasterPacket::Solve(field)) =>
                if run_solve(rx, tx, field, false) {
                    break;
                },
            Ok(MasterPacket::SolveDebug(field)) =>
                if run_solve(rx, tx, field, true) {
                    break;
                },
            Ok(MasterPacket::DebugTickAck(..)) =>
                (),
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
        let fp = &self.field.config.finish_area;
        fp.center.sq_dist(point) < fp.radius * fp.radius
    }

    fn trans_add_root(&mut self, empty_rtt: EmptyRandomTree<Point>) -> Result<RandomTree<Point>, NeverError> {
        Ok(empty_rtt.add_root(self.field.start))
    }

    fn trans_root_node(&mut self, rtt: &mut RandomTree<Point>) -> Result<RttNodeFocus, NeverError> {
        let root_ref = rtt.root();
        Ok(RttNodeFocus {
            node_ref: root_ref,
            goal_reached: self.goal_reached(&self.field.start),
        })
    }

    fn has_route(&self, rtt: &RandomTree<Point>, node_ref: &NodeRef, dst: &Point) -> bool {
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
    node_ref: NodeRef,
    goal_reached: bool,
}

impl RttNodeFocus {
    fn into_direct_path(self, rtt: RandomTree<Point>) -> Result<Vec<Point>, NeverError> {
        let mut rev_path: Vec<_> = rtt.into_path(self.node_ref).collect();
        rev_path.reverse();
        Ok(rev_path)
    }
}

fn run_solve(rx: &mpsc::Receiver<MasterPacket>, tx: &mpsc::Sender<SlavePacket>, field: Field, debug: bool) -> bool {
    let mut rng = rand::thread_rng();
    let mut debug_image = DebugImage {
        tick_id: 0,
        routes_segs: Vec::new(),
        sample_seg: SampleTry::None,
    };
    let mut last_ack = 0;
    let mut trans = Trans::new(field);

    let planner = rtt::PlannerInit::new(EmptyRandomTree::new());
    let planner = planner.add_root_ok(|empty_rtt| trans.trans_add_root(empty_rtt));
    let mut planner_node = planner.root_node_ok(|rtt: &mut _| trans.trans_root_node(rtt));
    loop {
        if planner_node.node_ref().goal_reached {
            let path = planner_node.into_path_ok(|rtt, focus: RttNodeFocus| focus.into_direct_path(rtt));;
            tx.send(SlavePacket::RouteDone(path)).ok();
            return false;
        }

        if debug {
            debug_image.routes_segs.clear();
            let mut visited: HashSet<(NodeRef, NodeRef)> = HashSet::new();

            let rtt = planner_node.rtt();
            let states = rtt.states();
            for (mut dst_node_ref, mut dst) in states.children {
                for (src_node_ref, src) in rtt.path_iter(&dst_node_ref).skip(1) {
                    let visited_key = (src_node_ref, dst_node_ref);
                    if visited.contains(&visited_key) {
                        break;
                    } else {
                        visited.insert(visited_key);
                        debug_image.routes_segs.push((src.clone(), dst.clone()));
                    }
                    dst_node_ref = src_node_ref;
                    dst = src;
                }
            }
        }

        let mut planner_ready_to_sample = planner_node.prepare_sample_ok(|_rtt: &mut _, _focus| Ok(()));
        loop {
            match rx.try_recv() {
                Ok(MasterPacket::Solve(..)) =>
                    (),
                Ok(MasterPacket::SolveDebug(..)) =>
                    (),
                Ok(MasterPacket::DebugTickAck(ack)) =>
                    last_ack = ack,
                Ok(MasterPacket::Terminate) =>
                    return true,
                Ok(MasterPacket::Abort) =>
                    return false,
                Err(mpsc::TryRecvError::Empty) =>
                    (),
                Err(mpsc::TryRecvError::Disconnected) =>
                    return true,
            }

            let planner_sample = planner_ready_to_sample.sample_ok(|_rtt: &mut _| {
                Ok(Point {
                    x: rng.gen_range(trans.field.config.field_area.0, trans.field.config.field_area.2),
                    y: rng.gen_range(trans.field.config.field_area.1, trans.field.config.field_area.3),
                })
            });

            let planner_closest = planner_sample.closest_to_sample_ok(|rtt: &mut RandomTree<Point>, sample: &Point| {
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
                Ok(closest.0)
            });

            let has_route = trans.has_route(planner_closest.rtt(), planner_closest.node_ref(), planner_closest.sample());

            if debug {
                let rtt = planner_closest.rtt();
                let closest_ref = planner_closest.node_ref();
                let src = rtt.get_state(closest_ref).clone();
                let dst = planner_closest.sample().clone();
                debug_image.sample_seg = if has_route {
                    SampleTry::Passable(src, dst)
                } else {
                    SampleTry::Blocked(src, dst)
                };
                if debug_image.tick_id == last_ack {
                    debug_image.tick_id += 1;
                    tx.send(SlavePacket::DebugTick(debug_image.clone())).ok();
                }
                ::std::thread::sleep(::std::time::Duration::from_millis(100));
            }

            if has_route {
                planner_node = planner_closest.has_transition_ok(|rtt: &mut RandomTree<Point>, node_ref: NodeRef, sample| {
                    let node_ref = rtt.expand(node_ref, sample);
                    let goal_reached = trans.goal_reached(rtt.get_state(&node_ref));
                    Ok(RttNodeFocus { node_ref, goal_reached, })
                });
                break;
            } else {
                planner_ready_to_sample = planner_closest.no_transition_ok(|_rtt: &mut _, _node_ref| Ok(()));
            }
        }
    }
}
