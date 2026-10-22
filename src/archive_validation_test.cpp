#include "formats/bigred_archive.h"

#include <cstdint>
#include <cstring>
#include <exception>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace {

using bigred::Blob;
using Files = std::unordered_map<std::string, Blob>;

int failures = 0;

void Fail(std::string_view test, std::string_view detail) {
    std::cerr << test << ": " << detail << '\n';
    ++failures;
}

void AppendU32Le(Blob& bytes, std::uint32_t value) {
    bytes.push_back(static_cast<std::uint8_t>(value));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8));
    bytes.push_back(static_cast<std::uint8_t>(value >> 16));
    bytes.push_back(static_cast<std::uint8_t>(value >> 24));
}

void AppendEntry(Blob& archive, std::string_view name, std::uint8_t method,
                 const Blob& payload, std::uint32_t uncompressed_size) {
    archive.insert(archive.end(), name.begin(), name.end());
    archive.push_back(0);
    AppendU32Le(archive, static_cast<std::uint32_t>(payload.size()));
    AppendU32Le(archive, uncompressed_size);
    archive.push_back(method);
    archive.insert(archive.end(), payload.begin(), payload.end());
}

Blob SingleEntry(std::string_view name, std::uint8_t method, const Blob& payload,
                 std::uint32_t uncompressed_size) {
    Blob archive;
    AppendEntry(archive, name, method, payload, uncompressed_size);
    return archive;
}

void ExpectRuntimeError(std::string_view test, const std::function<void()>& action) {
    try {
        action();
    } catch (const std::runtime_error&) {
        return;
    } catch (const std::exception& error) {
        Fail(test, std::string("wrong exception type: ") + error.what());
        return;
    } catch (...) {
        Fail(test, "wrong non-standard exception type");
        return;
    }
    Fail(test, "did not throw std::runtime_error");
}

void ExpectRejected(std::string_view test, const Blob& archive) {
    ExpectRuntimeError(test, [&archive] {
        (void)bigred::unpack_bytes(archive.data(), archive.size());
    });
}

void ExpectFile(std::string_view test, const Blob& archive, std::string_view name,
                const Blob& expected) {
    try {
        const Files files = bigred::unpack_bytes(archive.data(), archive.size());
        if (files.size() != 1) {
            Fail(test, "expected exactly one unpacked file");
            return;
        }
        const auto found = files.find(std::string(name));
        if (found == files.end()) {
            Fail(test, "expected filename was not returned");
            return;
        }
        if (found->second != expected) {
            Fail(test, "unpacked bytes differ from the control vector");
        }
    } catch (const std::exception& error) {
        Fail(test, std::string("valid archive threw: ") + error.what());
    } catch (...) {
        Fail(test, "valid archive threw a non-standard exception");
    }
}

void TestValidControls() {
    ExpectFile("stored/control", SingleEntry("RAW.BIN", 0, {0x00, 0x7f, 0x80, 0xff}, 4),
               "RAW.BIN", {0x00, 0x7f, 0x80, 0xff});
    ExpectFile("stored/empty", SingleEntry("EMPTY.BIN", 0, {}, 0), "EMPTY.BIN", {});

    // A two-byte literal run followed by a three-byte repeat run.
    const Blob rle_control{0x01, 'B', 'C', 0x80, 'A'};
    ExpectFile("rle/literal-and-repeat", SingleEntry("RLE.BIN", 1, rle_control, 5),
               "RLE.BIN", {'B', 'C', 'A', 'A', 'A'});
    Blob rle_max_literal{0x7f};
    Blob rle_max_literal_expected;
    for (unsigned i = 0; i < 128; ++i) {
        const auto value = static_cast<std::uint8_t>(i);
        rle_max_literal.push_back(value);
        rle_max_literal_expected.push_back(value);
    }
    ExpectFile("rle/max-literal", SingleEntry("RLE.BIN", 1, rle_max_literal, 128),
               "RLE.BIN", rle_max_literal_expected);
    ExpectFile("rle/max-repeat", SingleEntry("RLE.BIN", 1, {0xff, 'Z'}, 130),
               "RLE.BIN", Blob(130, 'Z'));

    // One literal followed by a three-byte back-reference to offset zero.
    for (std::uint8_t method = 2; method <= 6; ++method) {
        const unsigned offset_bits = 6u - method;
        const std::uint8_t run_flag = static_cast<std::uint8_t>(1u << offset_bits);
        const Blob lz_control{0x01, 'Q', 0x01, run_flag};
        const std::string name = "LZ" + std::to_string(method) + ".BIN";
        ExpectFile("lz/literal-and-back-reference-" + std::to_string(method),
                   SingleEntry(name, method, lz_control, 4), name,
                   {'Q', 'Q', 'Q', 'Q'});
    }

    Blob mixed;
    AppendEntry(mixed, "FIRST.BIN", 0, {1, 2, 3}, 3);
    AppendEntry(mixed, "SECOND.BIN", 1, rle_control, 5);
    mixed.push_back(0);
    try {
        const Files files = bigred::unpack_bytes(mixed.data(), mixed.size());
        const auto first = files.find("FIRST.BIN");
        const auto second = files.find("SECOND.BIN");
        if (files.size() != 2 || first == files.end() || second == files.end() ||
            first->second != Blob({1, 2, 3}) ||
            second->second != Blob({'B', 'C', 'A', 'A', 'A'})) {
            Fail("archive/multiple-records-and-eof", "control outputs were not preserved");
        }
    } catch (const std::exception& error) {
        Fail("archive/multiple-records-and-eof", std::string("valid archive threw: ") + error.what());
    }
}

void TestMalformedControls() {
    ExpectRuntimeError("archive/null-data", [] {
        (void)bigred::unpack_bytes(nullptr, 1);
    });
    // These cases require the decoder to reject before writing beyond the advertised size.
    // Do not run the default suite against the old vector out_len++ implementation.
    for (const std::uint8_t method : {std::uint8_t{7}, std::uint8_t{255}}) {
        ExpectRejected("method/unsupported-" + std::to_string(method),
                       SingleEntry("BAD.BIN", method, {}, 0));
    }

    ExpectRejected("stored/length-mismatch",
                   SingleEntry("BAD.BIN", 0, {0xaa, 0xbb}, 3));

    ExpectRejected("rle/advertised-output-overflow",
                   SingleEntry("BAD.BIN", 1, {0x80, 'A'}, 2));
    ExpectRejected("rle/output-short",
                   SingleEntry("BAD.BIN", 1, {0x01, 'A', 'B'}, 3));
    ExpectRejected("rle/truncated-repeat",
                   SingleEntry("BAD.BIN", 1, {0x80}, 3));
    ExpectRejected("rle/truncated-literal",
                   SingleEntry("BAD.BIN", 1, {0x01, 'A'}, 2));
    ExpectRejected("rle/impossible-allocation",
                   SingleEntry("BAD.BIN", 1, {0x80, 'A'}, 0xffffffffu));

    for (std::uint8_t method = 2; method <= 6; ++method) {
        const std::string suffix = std::to_string(method);
        ExpectRejected("lz/impossible-allocation-" + suffix,
                       SingleEntry("BAD.BIN", method, {0x01, 'A'}, 0xffffffffu));
        ExpectRejected("lz/literal-advertised-output-overflow-" + suffix,
                       SingleEntry("BAD.BIN", method, {0x03, 'A', 'B'}, 1));
        const unsigned offset_bits = 6u - method;
        const std::uint8_t run_flag = static_cast<std::uint8_t>(1u << offset_bits);
        ExpectRejected("lz/back-reference-advertised-output-overflow-" + suffix,
                       SingleEntry("BAD.BIN", method, {0x01, 'A', 0x01, run_flag}, 1));
        ExpectRejected("lz/truncated-literal-" + suffix,
                       SingleEntry("BAD.BIN", method, {0x01}, 1));
        ExpectRejected("lz/truncated-back-reference-" + suffix,
                       SingleEntry("BAD.BIN", method, {0x00, 0x00}, 3));
        ExpectRejected("lz/back-reference-before-output-" + suffix,
                       SingleEntry("BAD.BIN", method, {0x00, 0x01, 0x00}, 3));
        ExpectRejected("lz/output-short-" + suffix,
                       SingleEntry("BAD.BIN", method, {0x01, 'A'}, 2));
    }

    ExpectRejected("archive/unterminated-filename", Blob{'A', 'B', 'C'});
    ExpectRejected("archive/truncated-entry-header",
                   Blob{'A', 0, 1, 0, 0, 0});
    ExpectRejected("archive/truncated-payload", Blob{
        'A', 0,
        2, 0, 0, 0,
        2, 0, 0, 0,
        0,
        'X',
    });
    ExpectRejected("archive/trailing-partial-record", Blob{
        'A', 0,
        1, 0, 0, 0,
        1, 0, 0, 0,
        0,
        'X',
        'B',
    });
}

void PrintUsage() {
    std::cerr << "usage: archive_validation_test [--safe]\n"
                 "  --safe  run valid control vectors only\n"
                 "  default run valid and malformed vectors; the decoder must enforce output bounds\n";
}

}  // namespace

int main(int argc, char** argv) {
    bool safe_only = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--safe") == 0) {
            safe_only = true;
        } else {
            PrintUsage();
            return 2;
        }
    }

    TestValidControls();
    if (!safe_only) TestMalformedControls();

    if (failures != 0) {
        std::cerr << failures << " archive validation failure(s)\n";
        return 1;
    }
    std::cout << "archive validation passed ("
              << (safe_only ? "safe controls" : "controls and malformed bounds")
              << ")\n";
    return 0;
}
