//! Common state shared throughout the graphics system

use std::ffi::CStr;
use std::sync::Arc;

use ash::{
    version::{DeviceV1_0, InstanceV1_0},
    vk, Device,
};

use super::Core;

pub struct Base {
    pub core: Arc<Core>,
    pub physical: vk::PhysicalDevice,
    pub device: Arc<Device>,
    pub queue_family: u32,
    pub queue: vk::Queue,
    pub memory_properties: vk::PhysicalDeviceMemoryProperties,
    pub pipeline_cache: vk::PipelineCache,
    pub render_pass: vk::RenderPass,
    pub linear_sampler: vk::Sampler,
}

impl Drop for Base {
    fn drop(&mut self) {
        unsafe {
            self.device
                .destroy_pipeline_cache(self.pipeline_cache, None);
            self.device.destroy_render_pass(self.render_pass, None);
            self.device.destroy_sampler(self.linear_sampler, None);
            self.device.destroy_device(None);
        }
    }
}

impl Base {
    pub fn new(
        core: Arc<Core>,
        pipeline_cache_data: &[u8],
        device_exts: &[&CStr],
        mut device_filter: impl FnMut(vk::PhysicalDevice, u32) -> bool,
    ) -> Option<Self> {
        unsafe {
            let instance = &core.instance;
            let (physical, queue_family_index) = instance
                .enumerate_physical_devices()
                .unwrap()
                .into_iter()
                .find_map(|physical| {
                    instance
                        .get_physical_device_queue_family_properties(physical)
                        .iter()
                        .enumerate()
                        .find_map(|(queue_family_index, ref info)| {
                            let supports_graphic_and_surface =
                                info.queue_flags.contains(vk::QueueFlags::GRAPHICS)
                                    && device_filter(physical, queue_family_index as u32);
                            match supports_graphic_and_surface {
                                true => Some((physical, queue_family_index as u32)),
                                _ => None,
                            }
                        })
                })?;

            let device_exts = device_exts.iter().map(|x| x.as_ptr()).collect::<Vec<_>>();

            let device = Arc::new(
                instance
                    .create_device(
                        physical,
                        &vk::DeviceCreateInfo::builder()
                            .queue_create_infos(&[vk::DeviceQueueCreateInfo::builder()
                                .queue_family_index(queue_family_index)
                                .queue_priorities(&[1.0])
                                .build()])
                            .enabled_extension_names(&device_exts),
                        None,
                    )
                    .unwrap(),
            );
            let queue = device.get_device_queue(queue_family_index, 0);
            let memory_properties = instance.get_physical_device_memory_properties(physical);
            let pipeline_cache = device
                .create_pipeline_cache(
                    &vk::PipelineCacheCreateInfo::builder().initial_data(pipeline_cache_data),
                    None,
                )
                .unwrap();

            let render_pass = device
                .create_render_pass(
                    &vk::RenderPassCreateInfo::builder()
                        .attachments(&[
                            vk::AttachmentDescription {
                                format: COLOR_FORMAT,
                                samples: vk::SampleCountFlags::TYPE_1,
                                load_op: vk::AttachmentLoadOp::CLEAR,
                                store_op: vk::AttachmentStoreOp::STORE,
                                initial_layout: vk::ImageLayout::UNDEFINED,
                                final_layout: vk::ImageLayout::PRESENT_SRC_KHR,
                                ..Default::default()
                            },
                            vk::AttachmentDescription {
                                format: vk::Format::D32_SFLOAT,
                                samples: vk::SampleCountFlags::TYPE_1,
                                load_op: vk::AttachmentLoadOp::CLEAR,
                                store_op: vk::AttachmentStoreOp::DONT_CARE,
                                initial_layout: vk::ImageLayout::UNDEFINED,
                                final_layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                                ..Default::default()
                            },
                        ])
                        .subpasses(&[vk::SubpassDescription::builder()
                            .color_attachments(&[vk::AttachmentReference {
                                attachment: 0,
                                layout: vk::ImageLayout::COLOR_ATTACHMENT_OPTIMAL,
                            }])
                            .depth_stencil_attachment(&vk::AttachmentReference {
                                attachment: 1,
                                layout: vk::ImageLayout::DEPTH_STENCIL_ATTACHMENT_OPTIMAL,
                            })
                            .pipeline_bind_point(vk::PipelineBindPoint::GRAPHICS)
                            .build()])
                        .dependencies(&[vk::SubpassDependency {
                            src_subpass: vk::SUBPASS_EXTERNAL,
                            dst_subpass: 0,
                            src_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                            dst_stage_mask: vk::PipelineStageFlags::COLOR_ATTACHMENT_OUTPUT,
                            dst_access_mask: vk::AccessFlags::COLOR_ATTACHMENT_READ
                                | vk::AccessFlags::COLOR_ATTACHMENT_WRITE,
                            ..Default::default()
                        }]),
                    None,
                )
                .unwrap();

            let linear_sampler = device
                .create_sampler(
                    &vk::SamplerCreateInfo::builder()
                        .min_filter(vk::Filter::LINEAR)
                        .mag_filter(vk::Filter::LINEAR)
                        .mipmap_mode(vk::SamplerMipmapMode::NEAREST)
                        .address_mode_u(vk::SamplerAddressMode::CLAMP_TO_EDGE)
                        .address_mode_v(vk::SamplerAddressMode::CLAMP_TO_EDGE)
                        .address_mode_w(vk::SamplerAddressMode::CLAMP_TO_EDGE),
                    None,
                )
                .unwrap();

            Some(Self {
                core,
                physical,
                device,
                queue_family: queue_family_index,
                queue,
                memory_properties,
                pipeline_cache,
                render_pass,
                linear_sampler,
            })
        }
    }

    pub unsafe fn set_name<T: vk::Handle>(&self, object: T, name: &CStr) {
        let ex = match self.core.debug_utils.as_ref() {
            Some(x) => x,
            None => return,
        };
        ex.debug_utils_set_object_name(
            self.device.handle(),
            &vk::DebugUtilsObjectNameInfoEXT::builder()
                .object_type(T::TYPE)
                .object_handle(object.as_raw())
                .object_name(name),
        )
            .unwrap();
    }

}

pub const COLOR_FORMAT: vk::Format = vk::Format::B8G8R8A8_SRGB;