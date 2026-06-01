#!/usr/bin/env python3
"""Extract .proto schemas from an MCAP file and compile them to C++ with protoc.

Usage:
    python3 extract_and_compile_protos.py <mcap_file> <proto_dir> <cpp_dir> [protoc_path]

1. Reads the MCAP summary to find all protobuf schemas.
2. Reconstructs .proto text from embedded FileDescriptorProto objects.
3. Writes .proto files to <proto_dir>.
4. Runs protoc to generate .pb.h / .pb.cc into <cpp_dir>.
"""

import os
import subprocess
import sys
from pathlib import Path

from google.protobuf.descriptor_pb2 import FileDescriptorSet
from mcap.reader import make_reader

# ---- Proto text reconstruction from FileDescriptorProto --------------------

_TYPE_MAP = {
    1: "double", 2: "float", 3: "int64", 4: "uint64", 5: "int32",
    6: "fixed64", 7: "fixed32", 8: "bool", 9: "string", 10: "group",
    11: "message", 12: "bytes", 13: "uint32", 14: "enum", 15: "sfixed32",
    16: "sfixed64", 17: "sint32", 18: "sint64",
}

_LABEL_MAP = {1: "", 2: "", 3: "repeated "}


def _type_name(field) -> str:
    if field.type in (11, 14):
        tn = field.type_name
        return tn[1:] if tn.startswith(".") else tn
    return _TYPE_MAP.get(field.type, "unknown")


def _message_lines(msg, indent: int) -> list[str]:
    pfx = "  " * indent
    lines = [f"{pfx}message {msg.name} {{"]
    for enum in msg.enum_type:
        lines.append(f"{pfx}  enum {enum.name} {{")
        for val in enum.value:
            lines.append(f"{pfx}    {val.name} = {val.number};")
        lines += [f"{pfx}  }}", f"{pfx}"]
    for nested in msg.nested_type:
        if nested.options and nested.options.map_entry:
            continue
        lines += _message_lines(nested, indent + 1) + [f"{pfx}"]
    oneof_fields: dict[int, list] = {}
    regular = []
    for f in msg.field:
        if f.HasField("oneof_index"):
            oneof_fields.setdefault(f.oneof_index, []).append(f)
        else:
            regular.append(f)
    for f in regular:
        label = _LABEL_MAP.get(f.label, "")
        lines.append(f"{pfx}  {label}{_type_name(f)} {f.name} = {f.number};")
    for oi, fields in oneof_fields.items():
        name = msg.oneof_decl[oi].name
        lines.append(f"{pfx}  oneof {name} {{")
        for f in fields:
            lines.append(f"{pfx}    {_type_name(f)} {f.name} = {f.number};")
        lines.append(f"{pfx}  }}")
    lines.append(f"{pfx}}}")
    return lines


def _fd_to_proto(fd) -> str:
    lines = ['syntax = "proto3";', ""]
    if fd.package:
        lines += [f"package {fd.package};", ""]
    for dep in fd.dependency:
        lines.append(f'import "{dep}";')
    if fd.dependency:
        lines.append("")
    for enum in fd.enum_type:
        lines.append(f"enum {enum.name} {{")
        for val in enum.value:
            lines.append(f"  {val.name} = {val.number};")
        lines += ["}", ""]
    for msg in fd.message_type:
        lines += _message_lines(msg, 0) + [""]
    return "\n".join(lines)


# ---- Main ------------------------------------------------------------------

def main() -> None:
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <mcap_file> <proto_dir> <cpp_dir> [protoc]",
              file=sys.stderr)
        sys.exit(1)

    mcap_path = Path(sys.argv[1])
    proto_dir = Path(sys.argv[2])
    cpp_dir = Path(sys.argv[3])
    protoc = sys.argv[4] if len(sys.argv) > 4 else "protoc"

    # Step 1: extract .proto files from MCAP
    all_fds: dict[str, object] = {}
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        for schema in (summary.schemas or {}).values():
            if schema.encoding == "protobuf":
                fds = FileDescriptorSet()
                fds.ParseFromString(schema.data)
                for fd in fds.file:
                    if fd.name not in all_fds:
                        all_fds[fd.name] = fd

    proto_files: list[Path] = []
    for name, fd in sorted(all_fds.items()):
        out = proto_dir / name
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(_fd_to_proto(fd))
        proto_files.append(out)

    print(f"Extracted {len(proto_files)} .proto files to {proto_dir}")

    # Step 2: compile with protoc
    cpp_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        protoc,
        f"-I={proto_dir}",
        f"--cpp_out={cpp_dir}",
        *[str(p) for p in proto_files],
    ]
    subprocess.check_call(cmd)
    print(f"Generated C++ protobuf code in {cpp_dir}")


if __name__ == "__main__":
    main()
