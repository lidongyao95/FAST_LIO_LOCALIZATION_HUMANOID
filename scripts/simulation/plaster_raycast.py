"""Ray-cast the displayed plaster surface without traversing contact thin layers."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np


class PlasterRaycaster:
    def __init__(self, scene):
        scene = Path(scene).resolve()
        source = ET.parse(scene).getroot()
        surface = source.find("worldbody/geom[@name='plaster_surface']")
        if surface is None:
            raise ValueError('Missing continuous plaster surface')
        mesh = source.find(f"asset/mesh[@name='{surface.get('mesh')}']")
        if mesh is None or not mesh.get('file'):
            raise ValueError('Missing plaster mesh asset')
        root = ET.Element('mujoco', model='plaster surface ray queries')
        asset = ET.SubElement(root, 'asset')
        mesh = deepcopy(mesh)
        path = Path(mesh.get('file'))
        if not path.is_absolute():
            compiler = source.find('compiler')
            meshdir = compiler.get('meshdir', '') if compiler is not None else ''
            path = scene.parent / meshdir / path
        mesh.set('file', str(path.resolve()))
        asset.append(mesh)
        world = ET.SubElement(root, 'worldbody')
        self.mocap_names = []
        for geom in source.find('worldbody').findall('geom'):
            if geom.get('name') not in ('plaster_surface', 'assumed_flat_floor'):
                continue
            copy = deepcopy(geom)
            copy.set('group', '0')
            world.append(copy)
        for body in source.find('worldbody').findall('body'):
            if body.get('mocap') == 'true' and body.find('geom') is not None:
                copy = deepcopy(body)
                for geom in copy.iter('geom'):
                    geom.set('group', '0')
                world.append(copy)
                self.mocap_names.append(body.get('name'))
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        self.data = mujoco.MjData(self.model)
        self.groups = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        mujoco.mj_forward(self.model, self.data)

    def cast(self, physics_model, physics_data, origin, directions, geomids, distances):
        for name in self.mocap_names:
            source = int(physics_model.body(name).mocapid[0])
            target = int(self.model.body(name).mocapid[0])
            self.data.mocap_pos[target] = physics_data.mocap_pos[source]
            self.data.mocap_quat[target] = physics_data.mocap_quat[source]
        mujoco.mj_forward(self.model, self.data)
        mujoco.mj_multiRay(self.model, self.data, origin, directions.ravel(), self.groups,
                          1, -1, geomids, distances, None, len(directions), 30.)
