"""Read XYZ coordinates from an ASCII or uncompressed binary PCD."""
import numpy as np

FIELD_TYPES = {
    "F": {4: "<f4", 8: "<f8"},
    "I": {1: "i1", 2: "<i2", 4: "<i4", 8: "<i8"},
    "U": {1: "u1", 2: "<u2", 4: "<u4", 8: "<u8"},
}


def read_pcd(path):
    with path.open("rb") as stream:
        header = {}
        for line in stream:
            parts = line.decode("ascii").strip().split()
            if parts and not parts[0].startswith("#"):
                key = parts[0].upper()
                header[key] = parts[1:]
                if key == "DATA":
                    break
        else:
            raise ValueError("PCD header ended before DATA")

        fields = header["FIELDS"]
        sizes = list(map(int, header["SIZE"]))
        types = header["TYPE"]
        counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
        if len({len(fields), len(sizes), len(types), len(counts)}) != 1 or any(c <= 0 for c in counts):
            raise ValueError("Inconsistent PCD fields")
        count = int(header["POINTS"][0]) if "POINTS" in header else int(header["WIDTH"][0]) * int(header["HEIGHT"][0])
        encoding = header["DATA"][0].lower()
        if encoding == "ascii":
            offsets = dict(zip(fields, np.cumsum([0, *counts[:-1]])))
            xyz = np.loadtxt(stream, dtype=np.float32, usecols=[offsets[axis] for axis in "xyz"], ndmin=2)
        elif encoding == "binary":
            formats = []
            offsets = []
            size = 0
            for width, kind, columns in zip(sizes, types, counts):
                dtype = FIELD_TYPES[kind][width]
                formats.append((dtype, (columns,)) if columns != 1 else dtype)
                offsets.append(size)
                size += width * columns
            dtype = np.dtype({"names": fields, "formats": formats, "offsets": offsets, "itemsize": size})
            points = np.frombuffer(stream.read(), dtype=dtype)
            xyz = np.column_stack([points[axis].astype(np.float32) for axis in "xyz"])
        else:
            raise ValueError("Expected ASCII or uncompressed binary PCD")
        if xyz.shape != (count, 3):
            raise ValueError("PCD point count or XYZ field size mismatch")
        return xyz[np.isfinite(xyz).all(axis=1)]
