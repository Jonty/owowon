use nusb::DeviceInfo;
use owowon::device::{PID, VID};
use std::sync::RwLock;
use std::{collections::HashMap, sync::Arc, thread};

type DeviceList = Arc<RwLock<HashMap<String, DeviceInfo>>>;

pub struct DeviceSelector {
    list: DeviceList,
    #[allow(dead_code)]
    watch_thread: Option<thread::JoinHandle<()>>,
}

impl DeviceSelector {
    pub fn list(&self) -> &DeviceList {
        &self.list
    }

    pub fn new(update_ui: impl Fn() + Send + 'static) -> Result<Self, nusb::Error> {
        let list: DeviceList = Arc::new(RwLock::new(HashMap::new()));

        // Initial enumeration - add all matching devices
        for device in nusb::list_devices()? {
            if device.vendor_id() == VID as u16 && device.product_id() == PID as u16 {
                let id = device_id_string(&device);
                list.write().unwrap().insert(id, device);
            }
        }

        // Polling-based device monitoring in a background thread
        let list_clone = list.clone();
        let watch_thread = thread::spawn(move || {
            loop {
                thread::sleep(std::time::Duration::from_secs(1));

                // Get current device list
                let Ok(current_devices) = nusb::list_devices() else {
                    continue;
                };

                let mut new_list = HashMap::new();
                for device in current_devices {
                    if device.vendor_id() == VID as u16 && device.product_id() == PID as u16 {
                        let id = device_id_string(&device);
                        new_list.insert(id, device);
                    }
                }

                // Check if the list changed
                let mut list_guard = list_clone.write().unwrap();
                if new_list.len() != list_guard.len()
                    || !new_list.keys().all(|k| list_guard.contains_key(k))
                {
                    *list_guard = new_list;
                    drop(list_guard);
                    update_ui();
                }
            }
        });

        Ok(Self {
            list,
            watch_thread: Some(watch_thread),
        })
    }
}

impl Drop for DeviceSelector {
    fn drop(&mut self) {
        // Thread will be terminated when the program exits
        // We don't join it to avoid blocking on drop
    }
}

fn device_id_string(device: &DeviceInfo) -> String {
    // Create stable identifier from bus/device
    format!("{}:{}", device.bus_number(), device.device_address())
}
