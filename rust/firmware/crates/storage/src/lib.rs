//! Raw data partitions (see `partitions.csv`) and the AssistNow blob cache.

use core::ffi::CStr;
use esp_idf_svc::sys::{
    ESP_OK, esp_partition_erase_range, esp_partition_find_first, esp_partition_read,
    esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY, esp_partition_t,
    esp_partition_type_t_ESP_PARTITION_TYPE_DATA, esp_partition_write,
};
use winchy_gnss::assistnow::CacheHeader;

/// Flash erase granularity.
pub const SECTOR: usize = 4096;

/// Storage error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StorageError {
    /// Partition not in the partition table.
    NotFound,
    /// Access outside the partition.
    OutOfRange,
    /// ESP-IDF error code.
    Esp(i32),
}

/// A raw data partition found by label.
pub struct RawPartition {
    part: &'static esp_partition_t,
}

impl RawPartition {
    /// Finds the data partition `label`.
    ///
    /// # Errors
    /// [`StorageError::NotFound`].
    pub fn find(label: &CStr) -> Result<Self, StorageError> {
        // SAFETY: label is a valid C string; the returned pointer refers to the
        // static partition table and stays valid for the program's lifetime.
        let p = unsafe {
            esp_partition_find_first(
                esp_partition_type_t_ESP_PARTITION_TYPE_DATA,
                esp_partition_subtype_t_ESP_PARTITION_SUBTYPE_ANY,
                label.as_ptr(),
            )
        };
        // SAFETY: non-null pointers from esp_partition_find_first are 'static.
        unsafe { p.as_ref() }
            .map(|part| Self { part })
            .ok_or(StorageError::NotFound)
    }

    /// Partition size, bytes.
    #[must_use]
    pub fn size(&self) -> usize {
        self.part.size as usize
    }

    fn check(&self, offset: usize, len: usize) -> Result<(), StorageError> {
        if offset
            .checked_add(len)
            .is_some_and(|end| end <= self.size())
        {
            Ok(())
        } else {
            Err(StorageError::OutOfRange)
        }
    }

    /// Reads `buf.len()` bytes at `offset`.
    ///
    /// # Errors
    /// Range or flash errors.
    pub fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), StorageError> {
        self.check(offset, buf.len())?;
        // SAFETY: range checked above; buf is valid for writes of its length.
        esp(unsafe { esp_partition_read(self.part, offset, buf.as_mut_ptr().cast(), buf.len()) })
    }

    /// Erases `len` bytes at `offset` (both rounded out to whole sectors).
    ///
    /// # Errors
    /// Range or flash errors.
    pub fn erase(&self, offset: usize, len: usize) -> Result<(), StorageError> {
        let start = offset / SECTOR * SECTOR;
        let end = (offset + len).div_ceil(SECTOR) * SECTOR;
        self.check(start, end - start)?;
        // SAFETY: sector-aligned range inside the partition.
        esp(unsafe { esp_partition_erase_range(self.part, start, end - start) })
    }

    /// Writes `data` at `offset` (the range must be erased).
    ///
    /// # Errors
    /// Range or flash errors.
    pub fn write(&self, offset: usize, data: &[u8]) -> Result<(), StorageError> {
        self.check(offset, data.len())?;
        // SAFETY: range checked above; data is valid for reads of its length.
        esp(unsafe { esp_partition_write(self.part, offset, data.as_ptr().cast(), data.len()) })
    }
}

fn esp(code: i32) -> Result<(), StorageError> {
    if code == ESP_OK {
        Ok(())
    } else {
        Err(StorageError::Esp(code))
    }
}

/// AssistNow MGA blob cache: header in sector 0, blob from sector 1.
/// The header is written LAST, so an interrupted write never validates.
pub struct MgaCache {
    part: RawPartition,
}

impl MgaCache {
    /// Opens the `assist` partition.
    ///
    /// # Errors
    /// [`StorageError::NotFound`].
    pub fn open() -> Result<Self, StorageError> {
        Ok(Self {
            part: RawPartition::find(c"assist")?,
        })
    }

    /// The cached header, if a valid one is present.
    #[must_use]
    pub fn header(&self) -> Option<CacheHeader> {
        let mut b = [0u8; CacheHeader::SIZE];
        self.part.read(0, &mut b).ok()?;
        CacheHeader::decode(&b)
    }

    /// Loads and verifies the cached blob.
    #[must_use]
    pub fn load(&self) -> Option<(CacheHeader, Vec<u8>)> {
        let h = self.header()?;
        let mut blob = vec![0u8; h.len as usize];
        self.part.read(SECTOR, &mut blob).ok()?;
        h.verifies(&blob).then_some((h, blob))
    }

    /// Stores `blob` with its download time.
    ///
    /// # Errors
    /// Range (blob too large) or flash errors.
    pub fn store(&self, blob: &[u8], downloaded_at_s: i64) -> Result<(), StorageError> {
        self.part.check(SECTOR, blob.len())?;
        self.part.erase(0, SECTOR + blob.len())?;
        self.part.write(SECTOR, blob)?;
        self.part
            .write(0, &CacheHeader::for_blob(blob, downloaded_at_s).encode())
    }
}
