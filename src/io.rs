//! This module contains a few traits and blanket implementations
//! for (de)serializing and writing/reading numeric data to/from the disc.
//! To use it you should import the `Load` and `Store` traits and use the
//! `load_from` and `write_to` methods.
//!
//! # Example
//!
//! ```
//! use stud_rust_base::io::*;
//!
//! fn test() {
//!     let head = Vec::<u32>::load_from("head_file_name").expect("could not read head");
//!     let lat = Vec::<f32>::load_from("node_latitude_file_name").expect("could not read lat");
//!     head.write_to("output_file").expect("could not write head");
//! }
//! ```

use std::fs;
use std::fs::File;
use std::io::prelude::*;
use std::io::Result;
use std::mem;
use std::slice;

/// A trait which allows accessing the data of an object as a slice of bytes.
/// The bytes should represent a serialization of the object and allow
/// recreating it when reading these bytes again from the disk.
///
/// Do not use this Trait but rather the `Store` trait.
pub trait DataBytes {
    /// Should return the serialized object as a slice of bytes
    fn data_bytes(&self) -> &[u8];
}

/// A trait which mutably exposes the internal data of an object so that
/// a serialized object can be loaded from disk and written back into a precreated
/// object of the right size.
///
/// Do not use this Trait but rather the `Load` trait.
pub trait DataBytesMut {
    /// Should return a mutable slice of the internal data of the object
    fn data_bytes_mut(&mut self) -> &mut [u8];
}

impl<T: Copy> DataBytes for [T] {
    fn data_bytes(&self) -> &[u8] {
        let num_bytes = self.len() * mem::size_of::<T>();
        unsafe { slice::from_raw_parts(self.as_ptr() as *const u8, num_bytes) }
    }
}

impl<T: Copy> DataBytesMut for [T] {
    fn data_bytes_mut(&mut self) -> &mut [u8] {
        let num_bytes = self.len() * mem::size_of::<T>();
        unsafe { slice::from_raw_parts_mut(self.as_mut_ptr() as *mut u8, num_bytes) }
    }
}

impl<T: Copy> DataBytesMut for Vec<T> {
    fn data_bytes_mut(&mut self) -> &mut [u8] {
        let num_bytes = self.len() * mem::size_of::<T>();
        unsafe { slice::from_raw_parts_mut(self.as_mut_ptr() as *mut u8, num_bytes) }
    }
}

/// A trait which extends the `DataBytes` trait and exposes a method to write objects to disk.
pub trait Store : DataBytes {
    /// Writes the serialized object to the file with the given filename
    fn write_to(&self, filename: &str) -> Result<()> {
        File::create(filename)?.write_all(self.data_bytes())
    }
}

impl<T: DataBytes> Store for T {}
impl<T> Store for [T] where [T]: DataBytes {}

/// A trait to load serialized data back into objects.
pub trait Load : DataBytesMut + Sized {
    /// This method must create an object of the correct size for serialized data with the given number of bytes.
    /// It should not be necessary to call this method directly.
    fn new_with_bytes(num_bytes: usize) -> Self;

    /// This method will load serialized data from the disk, create an object of the appropriate size,
    /// deserialize the bytes into the object and return the object.
    fn load_from(filename: &str) -> Result<Self> {
        let metadata = fs::metadata(filename)?;
        let mut file = File::open(filename)?;

        let mut object = Self::new_with_bytes(metadata.len() as usize);
        assert_eq!(metadata.len() as usize, object.data_bytes_mut().len());
        file.read_exact(object.data_bytes_mut())?;

        Ok(object)
    }
}

impl<T: Default + Copy> Load for Vec<T> {
    fn new_with_bytes(num_bytes: usize) -> Self {
        assert_eq!(num_bytes % mem::size_of::<T>(), 0);
        let num_elements = num_bytes / mem::size_of::<T>();
        (0..num_elements).map(|_| T::default()).collect()
    }
}
