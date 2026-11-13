//! 歩行者のインスタンス描画。
//!
//! 1 人ずつ別のエンティティにすると、1 万人規模では描画の準備（抽出・可視判定・並べ替え・
//! バッチ作成）だけで 1 フレームに何十 ms もかかる（とくに 1 スレッドで動く Web 版）。
//! そこで歩行姿勢ごと（と遠くの簡易表示）に 1 つずつのエンティティだけを置き、
//! 各エンティティに「その姿勢で描く人の位置・向き・体格・色」の配列を持たせて、
//! 1 回の描画命令でまとめて描く（bevy の custom_shader_instancing の例と同じ仕組み）。
//!
//! 光の当たり方は太陽（平行光源 1 つ）と環境光の拡散反射だけで近似する。
//! 歩行者は影を落とさない。

use bevy::{
    core_pipeline::core_3d::Transparent3d,
    ecs::{
        query::QueryItem,
        system::{lifetimeless::*, SystemParamItem},
    },
    pbr::{
        MeshPipeline, MeshPipelineKey, RenderMeshInstances, SetMeshBindGroup, SetMeshViewBindGroup,
        ViewKeyCache,
    },
    prelude::*,
    render::{
        extract_component::{ExtractComponent, ExtractComponentPlugin},
        mesh::{
            allocator::MeshAllocator, MeshVertexBufferLayoutRef, RenderMesh, RenderMeshBufferInfo,
        },
        render_asset::RenderAssets,
        render_phase::{
            AddRenderCommand, DrawFunctions, PhaseItem, PhaseItemExtraIndex, RenderCommand,
            RenderCommandResult, SetItemPipeline, TrackedRenderPass, ViewSortedRenderPhases,
        },
        render_resource::*,
        renderer::{RenderDevice, RenderQueue},
        sync_world::MainEntity,
        view::ExtractedView,
        Render, RenderApp, RenderSet,
    },
};
use bytemuck::{Pod, Zeroable};

/// 1 人ぶんのインスタンスデータ（頂点バッファ 1 要素）
#[derive(Clone, Copy, Pod, Zeroable, Default, Debug)]
#[repr(C)]
pub struct PersonInstance {
    /// 足元の位置（ワールド座標）と向き [rad]（ローカルの +x を y 軸まわりにこの角度だけ回す）
    pub pos_heading: [f32; 4],
    /// 横幅の倍率, 身長の倍率, 下の組み合わせの番号, 未使用
    pub scale_lower: [f32; 4],
    /// 上着の色（リニア RGB）, 未使用
    pub top: [f32; 4],
}

/// このエンティティのメッシュ（ある歩行姿勢の人型）で描く人たち
#[derive(Component, Default, Clone)]
pub struct PersonInstances(pub Vec<PersonInstance>);

impl ExtractComponent for PersonInstances {
    type QueryData = &'static PersonInstances;
    type QueryFilter = ();
    type Out = Self;

    fn extract_component(item: QueryItem<'_, Self::QueryData>) -> Option<Self> {
        Some(item.clone())
    }
}

pub struct CrowdRenderPlugin;

#[derive(Resource, Clone)]
struct PersonShader(Handle<Shader>);

impl Plugin for CrowdRenderPlugin {
    fn build(&self, app: &mut App) {
        let shader = app
            .world_mut()
            .resource_mut::<Assets<Shader>>()
            .add(Shader::from_wgsl(shader_source(), "person_instancing.wgsl"));
        app.add_plugins(ExtractComponentPlugin::<PersonInstances>::default());
        let Some(render_app) = app.get_sub_app_mut(RenderApp) else {
            return;
        };
        render_app
            .insert_resource(PersonShader(shader))
            .add_render_command::<Transparent3d, DrawPeople>()
            .init_resource::<SpecializedMeshPipelines<PersonPipeline>>()
            .add_systems(
                Render,
                (
                    queue_people.in_set(RenderSet::QueueMeshes),
                    prepare_instance_buffers.in_set(RenderSet::PrepareResources),
                ),
            );
    }

    fn finish(&self, app: &mut App) {
        if let Some(render_app) = app.get_sub_app_mut(RenderApp) {
            render_app.init_resource::<PersonPipeline>();
        }
    }
}

fn shader_source() -> String {
    format!(
        "{}\n{}",
        crate::person::lowers_wgsl(),
        include_str!("person_instancing.wgsl")
    )
}

#[allow(clippy::too_many_arguments)]
fn queue_people(
    draw_functions: Res<DrawFunctions<Transparent3d>>,
    pipeline: Res<PersonPipeline>,
    mut pipelines: ResMut<SpecializedMeshPipelines<PersonPipeline>>,
    pipeline_cache: Res<PipelineCache>,
    meshes: Res<RenderAssets<RenderMesh>>,
    render_mesh_instances: Res<RenderMeshInstances>,
    view_key_cache: Res<ViewKeyCache>,
    groups: Query<(Entity, &MainEntity, &PersonInstances)>,
    mut phases: ResMut<ViewSortedRenderPhases<Transparent3d>>,
    views: Query<&ExtractedView>,
) {
    let draw = draw_functions.read().id::<DrawPeople>();
    for view in &views {
        let Some(phase) = phases.get_mut(&view.retained_view_entity) else {
            continue;
        };
        let Some(view_key) = view_key_cache.get(&view.retained_view_entity) else {
            continue;
        };
        let rangefinder = view.rangefinder3d();
        for (entity, main_entity, people) in &groups {
            if people.0.is_empty() {
                continue;
            }
            let Some(mesh_instance) = render_mesh_instances.render_mesh_queue_data(*main_entity)
            else {
                continue;
            };
            let Some(mesh) = meshes.get(mesh_instance.mesh_asset_id) else {
                continue;
            };
            let key =
                *view_key | MeshPipelineKey::from_primitive_topology(mesh.primitive_topology());
            let Ok(pipeline) = pipelines.specialize(&pipeline_cache, &pipeline, key, &mesh.layout)
            else {
                continue;
            };
            phase.add(Transparent3d {
                entity: (entity, *main_entity),
                pipeline,
                draw_function: draw,
                distance: rangefinder.distance_translation(&mesh_instance.translation),
                batch_range: 0..1,
                extra_index: PhaseItemExtraIndex::None,
                indexed: true,
            });
        }
    }
}

/// インスタンスデータの頂点バッファ。大きさが足りる間は作り直さずに中身だけ書き換える。
#[derive(Component)]
struct InstanceBuffer {
    buffer: Buffer,
    capacity: usize,
    length: usize,
}

fn prepare_instance_buffers(
    mut commands: Commands,
    mut query: Query<(Entity, &PersonInstances, Option<&mut InstanceBuffer>)>,
    render_device: Res<RenderDevice>,
    render_queue: Res<RenderQueue>,
) {
    for (entity, people, buf) in &mut query {
        let n = people.0.len();
        let bytes: &[u8] = bytemuck::cast_slice(&people.0);
        match buf {
            Some(mut b) if b.capacity >= n => {
                if n > 0 {
                    render_queue.write_buffer(&b.buffer, 0, bytes);
                }
                b.length = n;
            }
            _ => {
                // 増えたときに毎回作り直さないよう、余裕を持たせる
                let capacity = (n + n / 4).max(64);
                let mut data = vec![PersonInstance::default(); capacity];
                data[..n].copy_from_slice(&people.0);
                let buffer = render_device.create_buffer_with_data(&BufferInitDescriptor {
                    label: Some("person instance buffer"),
                    contents: bytemuck::cast_slice(&data),
                    usage: BufferUsages::VERTEX | BufferUsages::COPY_DST,
                });
                commands.entity(entity).insert(InstanceBuffer {
                    buffer,
                    capacity,
                    length: n,
                });
            }
        }
    }
}

#[derive(Resource)]
struct PersonPipeline {
    shader: Handle<Shader>,
    mesh_pipeline: MeshPipeline,
}

impl FromWorld for PersonPipeline {
    fn from_world(world: &mut World) -> Self {
        Self {
            shader: world.resource::<PersonShader>().0.clone(),
            mesh_pipeline: world.resource::<MeshPipeline>().clone(),
        }
    }
}

impl SpecializedMeshPipeline for PersonPipeline {
    type Key = MeshPipelineKey;

    fn specialize(
        &self,
        key: Self::Key,
        layout: &MeshVertexBufferLayoutRef,
    ) -> Result<RenderPipelineDescriptor, SpecializedMeshPipelineError> {
        let mut desc = self.mesh_pipeline.specialize(key, layout)?;
        desc.vertex.shader = self.shader.clone();
        // メッシュの頂点属性は位置(0)・法線(1)・頂点カラー(5)。インスタンスは 10〜12 番に置く
        let f4 = VertexFormat::Float32x4.size();
        desc.vertex.buffers.push(VertexBufferLayout {
            array_stride: size_of::<PersonInstance>() as u64,
            step_mode: VertexStepMode::Instance,
            attributes: (0..3)
                .map(|k| VertexAttribute {
                    format: VertexFormat::Float32x4,
                    offset: k as u64 * f4,
                    shader_location: 10 + k,
                })
                .collect(),
        });
        let frag = desc
            .fragment
            .as_mut()
            .expect("mesh pipeline has a fragment stage");
        frag.shader = self.shader.clone();
        // 不透明として描く（深度を書き、混ぜ合わせない）
        for t in frag.targets.iter_mut().flatten() {
            t.blend = None;
        }
        if let Some(ds) = desc.depth_stencil.as_mut() {
            ds.depth_write_enabled = true;
        }
        Ok(desc)
    }
}

type DrawPeople = (
    SetItemPipeline,
    SetMeshViewBindGroup<0>,
    SetMeshBindGroup<1>,
    DrawMeshInstanced,
);

struct DrawMeshInstanced;

impl<P: PhaseItem> RenderCommand<P> for DrawMeshInstanced {
    type Param = (
        SRes<RenderAssets<RenderMesh>>,
        SRes<RenderMeshInstances>,
        SRes<MeshAllocator>,
    );
    type ViewQuery = ();
    type ItemQuery = Read<InstanceBuffer>;

    #[inline]
    fn render<'w>(
        item: &P,
        _view: (),
        instance_buffer: Option<&'w InstanceBuffer>,
        (meshes, render_mesh_instances, mesh_allocator): SystemParamItem<'w, '_, Self::Param>,
        pass: &mut TrackedRenderPass<'w>,
    ) -> RenderCommandResult {
        let mesh_allocator = mesh_allocator.into_inner();
        let Some(mesh_instance) = render_mesh_instances.render_mesh_queue_data(item.main_entity())
        else {
            return RenderCommandResult::Skip;
        };
        let Some(gpu_mesh) = meshes.into_inner().get(mesh_instance.mesh_asset_id) else {
            return RenderCommandResult::Skip;
        };
        let Some(instance_buffer) = instance_buffer.filter(|b| b.length > 0) else {
            return RenderCommandResult::Skip;
        };
        let Some(vertex_slice) = mesh_allocator.mesh_vertex_slice(&mesh_instance.mesh_asset_id)
        else {
            return RenderCommandResult::Skip;
        };
        pass.set_vertex_buffer(0, vertex_slice.buffer.slice(..));
        pass.set_vertex_buffer(1, instance_buffer.buffer.slice(..));
        let instances = 0..instance_buffer.length as u32;
        match &gpu_mesh.buffer_info {
            RenderMeshBufferInfo::Indexed {
                index_format,
                count,
            } => {
                let Some(index_slice) =
                    mesh_allocator.mesh_index_slice(&mesh_instance.mesh_asset_id)
                else {
                    return RenderCommandResult::Skip;
                };
                pass.set_index_buffer(index_slice.buffer.slice(..), 0, *index_format);
                pass.draw_indexed(
                    index_slice.range.start..(index_slice.range.start + count),
                    vertex_slice.range.start as i32,
                    instances,
                );
            }
            RenderMeshBufferInfo::NonIndexed => {
                pass.draw(vertex_slice.range, instances);
            }
        }
        RenderCommandResult::Success
    }
}
