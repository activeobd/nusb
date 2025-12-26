use std::ffi::c_void;
use std::mem::{self, ManuallyDrop, MaybeUninit};
use std::ptr::null;

use log::debug;
use windows_sys::Win32::{
    Devices::Usb::USBD_ISO_PACKET_DESCRIPTOR,
    Foundation::{
        GetLastError, ERROR_DEVICE_NOT_CONNECTED, ERROR_FILE_NOT_FOUND, ERROR_GEN_FAILURE,
        ERROR_NO_SUCH_DEVICE, ERROR_OPERATION_ABORTED, ERROR_REQUEST_ABORTED, ERROR_SEM_TIMEOUT,
        ERROR_SUCCESS, ERROR_TIMEOUT,
    },
    System::IO::{GetOverlappedResult, OVERLAPPED},
};

use crate::transfer::{internal::notify_completion, Buffer, Completion, IsochBufferPacketDescriptor, Direction, TransferError};

use super::Interface;

#[repr(C)]
pub struct TransferData {
    // first member of repr(C) struct; can cast pointer between types
    // overlapped.Internal contains the status
    // overlapped.InternalHigh contains the number of bytes transferred
    pub(crate) overlapped: OVERLAPPED,
    pub(crate) buf: *mut u8,
    pub(crate) capacity: u32,
    pub(crate) request_len: u32,
    pub(crate) endpoint: u8,
    pub(crate) error_from_submit: Result<(), TransferError>,

    // Isochronous transfer relevant fields
    // Direction::In (Read): total number of isochronous packets required to hold the transfer buffer
    pub(crate) isoch_in_packets_num: Option<usize>,
    // Direction::In (Read): the number of elements in the slice should match isoch_in_packets_num
    pub(crate) isoch_packet_descriptors: Box<[MaybeUninit<USBD_ISO_PACKET_DESCRIPTOR>]>,
    // OS buffer handle (opaque)
    pub(crate) isoch_os_handle: *const c_void,
}

unsafe impl Send for TransferData {}
unsafe impl Sync for TransferData {}

impl TransferData {
    pub(crate) fn new(endpoint: u8, isoch_in_packets_num: Option<usize>) -> TransferData {
        let mut empty = ManuallyDrop::new(Vec::with_capacity(0));

        TransferData {
            overlapped: unsafe { mem::zeroed() },
            buf: empty.as_mut_ptr(),
            capacity: 0,
            request_len: 0,
            endpoint,
            error_from_submit: Ok(()),
            isoch_in_packets_num: isoch_in_packets_num,
            isoch_packet_descriptors: vec![MaybeUninit::<USBD_ISO_PACKET_DESCRIPTOR>::zeroed(); isoch_in_packets_num.unwrap_or(0)].into_boxed_slice(),
            isoch_os_handle: null(),
        }
    }

    #[inline]
    pub fn actual_len(&self) -> usize {
        self.overlapped.InternalHigh
    }

    pub fn set_buffer(&mut self, buf: Buffer) {
        debug_assert!(self.capacity == 0);
        let buf = ManuallyDrop::new(buf);
        self.capacity = buf.capacity;
        self.buf = buf.ptr;
        self.overlapped.InternalHigh = 0;
        self.isoch_os_handle = buf.os_handle;
        self.request_len = match Direction::from_address(self.endpoint) {
            Direction::Out => buf.len,
            Direction::In => buf.requested_len,
        };
    }

    pub fn take_completion(&mut self, intf: &Interface) -> Completion {
        let mut actual_len: u32 = 0;

        let status = self.error_from_submit.and_then(|()| {
            unsafe { GetOverlappedResult(intf.handle, &self.overlapped, &mut actual_len, 0) };

            match unsafe { GetLastError() } {
                ERROR_SUCCESS => Ok(()),
                ERROR_GEN_FAILURE => Err(TransferError::Stall),
                ERROR_REQUEST_ABORTED
                | ERROR_TIMEOUT
                | ERROR_SEM_TIMEOUT
                | ERROR_OPERATION_ABORTED => Err(TransferError::Cancelled),
                ERROR_FILE_NOT_FOUND | ERROR_DEVICE_NOT_CONNECTED | ERROR_NO_SUCH_DEVICE => {
                    Err(TransferError::Disconnected)
                }
                e => Err(TransferError::Unknown(e)),
            }
        });

        let isoch_packets_list = match self.isoch_in_packets_num {
            Some(_) => {
                // produce packet descriptors list (library struct)
                let isoch_packets_list: Vec<IsochBufferPacketDescriptor> = self
                    .isoch_packet_descriptors
                    .iter()
                    // omit bad status packets
                    .filter(|desc| unsafe { desc.assume_init().Status == 0 })
                    // omit empty packets
                    .filter(|desc| unsafe { desc.assume_init().Length > 0 })
                    // omit invalid packets
                    .filter(|desc| unsafe { desc.assume_init().Offset + desc.assume_init().Length <= self.request_len })
                    // OS descriptor -> library descriptor
                    .map(|desc| {
                        IsochBufferPacketDescriptor {
                            offset: unsafe { desc.assume_init().Offset as usize },
                            length: unsafe { desc.assume_init().Length as usize },
                        }
                    })
                    .collect();

                // override actual buffer len
                actual_len = isoch_packets_list.iter().map(|x| x.length as u32).sum();

                Some(isoch_packets_list)
            },
            None => None,
        };

        let mut empty = ManuallyDrop::new(Vec::new());
        let ptr = mem::replace(&mut self.buf, empty.as_mut_ptr());
        let capacity = mem::replace(&mut self.capacity, 0);
        let isoch_os_handle = mem::replace(&mut self.isoch_os_handle, null());
        let len = match Direction::from_address(self.endpoint) {
            Direction::Out => self.request_len,
            Direction::In => match self.isoch_in_packets_num {
                Some(_) => self.request_len,    // isochronous transfer: return the entire buffer
                None => actual_len,             // normal transfer: return first bytes of actual len (continuous data)
            },
        };
        let requested_len = mem::replace(&mut self.request_len, 0);
        self.overlapped.InternalHigh = 0;

        Completion {
            status,
            actual_len: actual_len as usize,
            buffer: Buffer {
                ptr,
                len,
                requested_len,
                capacity,
                allocator: crate::transfer::Allocator::Default,
                os_handle: isoch_os_handle,
            },
            isoch_packets_list,
        }
    }
}

impl Drop for TransferData {
    fn drop(&mut self) {
        unsafe {
            drop(Vec::from_raw_parts(self.buf, 0, self.capacity as usize));
        }
    }
}

pub(super) fn handle_event(completion: *mut OVERLAPPED) {
    let t = completion as *mut TransferData;
    {
        let transfer = unsafe { &mut *t };

        debug!(
            "Transfer {t:?} on endpoint {:02x} complete: status {}, {} bytes",
            transfer.endpoint,
            transfer.overlapped.Internal,
            transfer.actual_len(),
        );
    }
    unsafe { notify_completion::<TransferData>(t) }
}
