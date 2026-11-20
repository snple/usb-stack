//! The classes that drive a device through the host stack.
//!
//! This mirrors [`crate::device::class`], which holds the classes that serve a
//! host instead.
//!
//! | Class | Interface |
//! |---|---|
//! | [`hid::Keyboard`] | HID boot-protocol keyboard |

pub mod hid;
