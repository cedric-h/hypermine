use std::ffi::CStr;
use std::os::raw::c_void;
use std::ptr;
use std::slice;

use ash::extensions::ext::DebugUtils;
use ash::version::{EntryV1_0, InstanceV1_0};
use ash::{vk, Entry, Instance};
use tracing::{debug, error, info, trace, warn};

use common::defer;

pub struct Core {
    pub entry: Entry,
    pub instance: Instance,

    pub debug_utils: Option<DebugUtils>,
    messenger: vk::DebugUtilsMessengerEXT,
}

impl Drop for Core {
    fn drop(&mut self) {
        unsafe {
            if let Some(ref utils) = self.debug_utils {
                utils.destroy_debug_utils_messenger(self.messenger, None);
            }
            self.instance.destroy_instance(None);
        }
    }
}

impl Core {
    pub fn new(exts: &[&CStr]) -> Self {
        let entry = Entry::new().unwrap();

        unsafe {
            let supported_exts = entry.enumerate_instance_extension_properties().unwrap();
            let has_debug = supported_exts
                .iter()
                .any(|x| CStr::from_ptr(x.extension_name.as_ptr()) == DebugUtils::name());

            let mut exts = exts.iter().map(|x| x.as_ptr()).collect::<Vec<_>>();
            exts.push(b"VK_KHR_get_physical_device_properties2\0".as_ptr() as _);
            if has_debug {
                exts.push(DebugUtils::name().as_ptr());
            } else {
                info!("vulkan debugging unavailable");
            }

            let name = cstr!("hypermine");

            let instance = entry
                .create_instance(
                    &vk::InstanceCreateInfo::builder()
                        .application_info(
                            &vk::ApplicationInfo::builder()
                                .application_name(name)
                                .application_version(0)
                                .engine_name(name)
                                .engine_version(0)
                                .api_version(ash::vk_make_version!(1, 1, 0)),
                        )
                        .enabled_extension_names(&exts),
                    None,
                )
                .unwrap();
            let instance_guard = defer(|| instance.destroy_instance(None));
            let messenger_guard;
            let debug_utils;
            let messenger;
            if has_debug {
                let utils = DebugUtils::new(&entry, &instance);
                messenger = utils
                    .create_debug_utils_messenger(
                        &vk::DebugUtilsMessengerCreateInfoEXT::builder()
                            .message_severity(
                                vk::DebugUtilsMessageSeverityFlagsEXT::ERROR
                                    | vk::DebugUtilsMessageSeverityFlagsEXT::WARNING
                                    | vk::DebugUtilsMessageSeverityFlagsEXT::INFO
                                    | vk::DebugUtilsMessageSeverityFlagsEXT::VERBOSE,
                            )
                            .message_type(
                                vk::DebugUtilsMessageTypeFlagsEXT::GENERAL
                                    | vk::DebugUtilsMessageTypeFlagsEXT::VALIDATION
                                    | vk::DebugUtilsMessageTypeFlagsEXT::PERFORMANCE,
                            )
                            .pfn_user_callback(Some(messenger_callback))
                            .user_data(ptr::null_mut()),
                        None,
                    )
                    .unwrap();
                debug_utils = Some(utils);
                messenger_guard = Some(defer(|| {
                    debug_utils
                        .as_ref()
                        .unwrap()
                        .destroy_debug_utils_messenger(messenger, None)
                }));
            } else {
                debug_utils = None;
                messenger_guard = None;
                messenger = vk::DebugUtilsMessengerEXT::null();
            }

            instance_guard.cancel();
            messenger_guard.map(|x| x.cancel());
            Self {
                entry,
                instance,
                debug_utils,
                messenger,
            }
        }
    }
}

unsafe extern "system" fn messenger_callback(
    message_severity: vk::DebugUtilsMessageSeverityFlagsEXT,
    _message_types: vk::DebugUtilsMessageTypeFlagsEXT,
    p_data: *const vk::DebugUtilsMessengerCallbackDataEXT,
    _p_user_data: *mut c_void,
) -> vk::Bool32 {
    let data = &*p_data;
    let msg_id = if data.p_message_id_name.is_null() {
        "".into()
    } else {
        CStr::from_ptr(data.p_message_id_name).to_string_lossy()
    };
    let msg = CStr::from_ptr(data.p_message).to_string_lossy();
    let queue_labels = fmt_labels(data.p_queue_labels, data.queue_label_count);
    let cmd_labels = fmt_labels(data.p_cmd_buf_labels, data.cmd_buf_label_count);
    let objects = slice::from_raw_parts(data.p_objects, data.object_count as usize)
        .iter()
        .map(|obj| {
            if obj.p_object_name.is_null() {
                format!("{:?} {:x}", obj.object_type, obj.object_handle)
            } else {
                format!("{:?} {:x} {}", obj.object_type, obj.object_handle, msg_id)
            }
        })
        .collect::<Vec<_>>()
        .join(", ");
    if message_severity >= vk::DebugUtilsMessageSeverityFlagsEXT::ERROR {
        error!(target: "vulkan", id = %msg_id, number = data.message_id_number, queue_labels = %queue_labels, cmd_labels = %cmd_labels, objects = %objects, "{}", msg);
    } else if message_severity >= vk::DebugUtilsMessageSeverityFlagsEXT::WARNING {
        warn! (target: "vulkan", id = %msg_id, number = data.message_id_number, queue_labels = %queue_labels, cmd_labels = %cmd_labels, objects = %objects, "{}", msg);
    } else if message_severity >= vk::DebugUtilsMessageSeverityFlagsEXT::INFO {
        debug!(target: "vulkan", id = %msg_id, number = data.message_id_number, queue_labels = %queue_labels, cmd_labels = %cmd_labels, objects = %objects, "{}", msg);
    } else {
        trace!(target: "vulkan", id = %msg_id, number = data.message_id_number, queue_labels = %queue_labels, cmd_labels = %cmd_labels, objects = %objects, "{}", msg);
    }
    vk::FALSE
}

unsafe fn fmt_labels(ptr: *const vk::DebugUtilsLabelEXT, count: u32) -> String {
    slice::from_raw_parts(ptr, count as usize)
        .iter()
        .map(|label| {
            CStr::from_ptr(label.p_label_name)
                .to_string_lossy()
                .into_owned()
        })
        .collect::<Vec<_>>()
        .join(", ")
}