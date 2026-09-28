"""Package an RP2040 flash binary as UF2; Python standard library only."""
import argparse
import pathlib
import struct


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    args = parser.parse_args()
    data = args.binary.read_bytes()
    if not 256 <= len(data) <= 2 * 1024 * 1024:
        parser.error("expected an RP2040 flash binary including boot2, at most 2 MiB")
    count = (len(data) + 255) // 256
    with args.output.open("wb") as out:
        for index in range(count):
            chunk = data[index * 256:(index + 1) * 256].ljust(256, b"\0")
            out.write(struct.pack("<8I", 0x0A324655, 0x9E5D5157, 0x2000,
                                  0x10000000 + index * 256, 256, index, count, 0xE48BFF56))
            out.write(chunk + bytes(220))
            out.write(struct.pack("<I", 0x0AB16F30))
    print(f"{count} RP2040 UF2 blocks -> {args.output}")


if __name__ == "__main__":
    main()
