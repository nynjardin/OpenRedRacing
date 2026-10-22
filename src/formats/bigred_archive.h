// Unpacker for Big Red Racing's archive format (.ENV / .WAP / .LEV / .VHC / .SCN all share it).
// ⚠️ `.SCN` was only recognised as this same container on
// 2026-09-22, after three sessions spent tokenizing its *compressed* bytes as if they were the
// UI script they hold; the reflex to keep is to try this unpacker on any unfamiliar extension
// before theorising about its format.
// C++ port of tools/bigred_archive.py (itself a port of research/red-archive's unpack()).
// Format: sequential entries with no header, each `name\0` + 4-byte compressed size (LE) +
// 4-byte uncompressed size (LE) + 1-byte compression type + compressed data.
// Compression type 0 = raw (sizes must match), 1 = RLE, 2-6 = LZSS variants
// (offset bits = 6 - type).
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace bigred {

using Blob = std::vector<uint8_t>;

// Unpacks raw archive bytes into {filename: file_data}. Throws std::runtime_error on any
// format inconsistency (unsupported compression type, truncated data, declared-size overflow,
// or a malformed LZSS/RLE stream). Every chunk and decompressed write is bounds checked.
std::unordered_map<std::string, Blob> unpack_bytes(const uint8_t* data, size_t size);

// Unpacks any archive file. Under the game directory, use game_archive() (game_files.h) instead:
// it builds the path and unpacks each archive once.
std::unordered_map<std::string, Blob> unpack_file(const std::string& path);

// Reads a whole file whose path is UTF-8. Returns an empty blob when it is not there.
//
// ⚠️ USE THIS RATHER THAN fopen/ifstream for anything under the game directory. On Windows the
// narrow-char file APIs interpret bytes through the ANSI codepage, not UTF-8, so a source
// literal containing an accent - an install path such as
// `C:\Jeux\Éditions\...` - gets mojibake'd and every open silently fails. The archive loader
// hit that first; the sound bank hit it again the day it was written, and every file resolved
// as ABSENT. Same conversion, one place.
Blob read_file_utf8(const std::string& path_utf8);
bool file_exists_utf8(const std::string& path_utf8);

// Writes `data` to `path_utf8`, overwriting it. Creates no directories - call
// create_directories_utf8() first. Returns false on any failure (read-only volume, missing
// parent directory, ...), which every caller must treat as "the save did not happen" rather
// than crash: a profile or a settings file that fails to write should leave the player where
// they were, not take the game down.
bool write_file_utf8(const std::string& path_utf8, const Blob& data);

// mkdir -p for a UTF-8 path: creates every missing component. True if the directory exists
// afterwards, whether it already did or was just created.
bool create_directories_utf8(const std::string& dir_utf8);

} // namespace bigred
