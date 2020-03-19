use std::sync::Arc;
use std::time::Duration;

use tracing::{debug, error};

use crate::{graphics::lru_table::SlotId, Config, Net};
use common::{
    dodeca,
    graph::{Graph, NodeId},
    math,
    world::{Material, SUBDIVISION_FACTOR},
};

/// Game state
pub struct Sim {
    view_reference: NodeId,
    view: na::Matrix4<f64>,
    velocity: na::Vector3<f64>,
    net: Net,
    config: Arc<Config>,
    pub graph: Graph<bool, Cube>,
}

impl Sim {
    pub fn new(net: Net, config: Arc<Config>) -> Self {
        let mut result = Self {
            view_reference: NodeId::ROOT,
            view: na::Matrix4::identity(),
            velocity: na::zero(),
            net,
            graph: Graph::new(),
            config,
        };

        result.populate_node(NodeId::ROOT);
        result
            .graph
            .ensure_nearby(NodeId::ROOT, result.config.view_distance);
        result.populate_fresh_nodes();

        result
    }

    pub fn rotate(&mut self, delta: na::Vector3<f64>) {
        self.view = self.view
            * na::Matrix4::from_axis_angle(&na::Vector3::y_axis(), -delta.x)
            * na::Matrix4::from_axis_angle(&na::Vector3::x_axis(), -delta.y)
            * na::Matrix4::from_axis_angle(&na::Vector3::z_axis(), -delta.z);
    }

    pub fn velocity(&mut self, v: na::Vector3<f64>) {
        self.velocity = v;
    }

    pub fn step(&mut self, dt: Duration) {
        while let Ok(msg) = self.net.incoming.try_recv() {
            use crate::net::Message::*;
            match msg {
                ConnectionLost(e) => {
                    error!("connection lost: {}", e);
                }
                _ => {
                    debug!("unimplemented: {:?}", msg);
                }
            }
        }

        let (dir, rate) = na::Unit::new_and_get(na::convert::<_, na::Vector3<f64>>(self.velocity));
        let distance = rate * dt.as_secs_f64() * 0.5;
        let view = math::renormalize_isometry(&self.view) * math::translate_along(&dir, distance);
        let (reference, view) = self.graph.normalize_transform(self.view_reference, view);
        if reference != self.view_reference {
            debug!("moved to node {:?}", reference);
        }
        self.view_reference = reference;
        self.view = view;

        self.graph
            .ensure_nearby(self.view_reference, self.config.view_distance);
        self.populate_fresh_nodes();
    }

    pub fn view_reference(&self) -> NodeId {
        self.view_reference
    }

    pub fn view(&self) -> &na::Matrix4<f64> {
        &self.view
    }

    fn populate_fresh_nodes(&mut self) {
        let fresh = self.graph.fresh().to_vec();
        self.graph.clear_fresh();
        for &node in &fresh {
            self.populate_node(node);
        }
        for &node in &fresh {
            for cube in self.graph.cubes_at(node) {
                self.populate_cube(node, cube);
            }
        }
        self.graph.clear_fresh();
    }

    fn populate_node(&mut self, node: NodeId) {
        use common::dodeca::Side;

        *self.graph.get_mut(node) = Some(match self.graph.parent(node) {
            None => true,
            Some(x) => {
                let parent_solid = self
                    .graph
                    .get(self.graph.neighbor(node, x).unwrap())
                    .unwrap();
                if x == Side::A {
                    !parent_solid
                } else {
                    parent_solid
                }
            }
        });
    }

    fn populate_cube(&mut self, node: NodeId, cube: dodeca::Vertex) {
        let contains_border = cube.canonical_sides().iter().any(|&x| x == dodeca::Side::A);

        let voxels = if contains_border {
            let mut data = (0..(SUBDIVISION_FACTOR + 2).pow(3))
                .map(|_| Material::Void)
                .collect::<Vec<_>>()
                .into_boxed_slice();

            const S: usize = SUBDIVISION_FACTOR;
            for z in 0..S {

                for y in 0..S {
                    for x in 0..S {

                        let i = (x + 1)
                            + (y + 1) * (S + 2)
                            + (z + 1) * (S + 2).pow(2);

                        const R: usize = S - 1;
                        let sides = 
                               ((x == 0 || x == R) && (y == 0 || y == R))
                            || ((y == 0 || y == R) && (z == 0 || z == R))
                            || ((z == 0 || z == R) && (x == 0 || x == R));

                        let y_wrap = ((S as i32/2 - y as i32).abs() as f32/2.0).round() as usize;
                        let center_x = ((S as i32/2 - x as i32).abs() as f32/2.0).round() as usize == y_wrap
                            && y_wrap == ((S as i32/2 - z as i32).abs() as f32/2.0).round() as usize;

                        data[i] = match (sides, center_x) {
                            (true, false) => Material::Stone,
                            (false, true) => Material::Sand,
                            (true, true)  => Material::Dirt,
                            _             => Material::Void,
                        };
                    }
                }
            }
            VoxelData::Dense(data)
        } else {
            VoxelData::Empty
        };
        *self.graph.get_cube_mut(node, cube) = Some(Cube {
            surface: None,
            voxels,
        });
    }
}

pub struct Cube {
    pub surface: Option<SlotId>,
    pub voxels: VoxelData,
}

pub enum VoxelData {
    Empty,
    Dense(Box<[Material]>),
}
