"""Pack the platter STLs into ONE Bambu Studio object with 3 parts (Bambu-native 3MF layout):
white inlay layer + white top -> WHITE filament slot, black blobs -> BLACK filament slot. 0.2 mm layers.
python make_3mf.py [white_slot] [black_slot]   (AMS slot numbers, default 1 2)"""
import sys, uuid, zipfile
import xml.etree.ElementTree as ET
import trimesh

WHITE, BLACK = (int(a) for a in (sys.argv[1:3] if len(sys.argv) > 2 else (1, 2)))
PARTS = [("white_pattern_layer", "platter_white_inlay.stl", WHITE),
         ("white_top", "platter_white_top.stl", WHITE),
         ("black_blobs", "platter_black.stl", BLACK)]
CENTER = (128, 128)  # mm on the bed
U = lambda: str(uuid.uuid4())
I4 = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"
NS = ('xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" '
      'xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" '
      'xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p"')

meshes, n_tris = [], 0
for i, (name, f, ext) in enumerate(PARTS, 1):
    m = trimesh.load(f)
    m.merge_vertices()
    assert m.is_watertight, f
    vx = "".join(f'<vertex x="{x:.4f}" y="{y:.4f}" z="{z:.4f}"/>' for x, y, z in m.vertices)
    tx = "".join(f'<triangle v1="{a}" v2="{b}" v3="{c}"/>' for a, b, c in m.faces)
    meshes.append(f'<object id="{i}" p:UUID="{U()}" type="model"><mesh><vertices>{vx}</vertices>'
                  f'<triangles>{tx}</triangles></mesh></object>')
    n_tris += len(m.faces)
OBJ = len(PARTS) + 1  # assembly object id

parts_model = f'<?xml version="1.0" encoding="UTF-8"?>\n<model unit="millimeter" xml:lang="en-US" {NS}>' \
              f'<metadata name="BambuStudio:3mfVersion">1</metadata><resources>{"".join(meshes)}</resources><build/></model>'
comps = "".join(f'<component p:path="/3D/Objects/object_1.model" objectid="{i}" p:UUID="{U()}" '
                f'transform="1 0 0 0 1 0 0 0 1 0 0 0"/>' for i in range(1, OBJ))
tr = f"1 0 0 0 1 0 0 0 1 {CENTER[0]} {CENTER[1]} 0"
main_model = f'''<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" {NS}>
<metadata name="Application">BambuStudio-02.00.00.00</metadata>
<metadata name="BambuStudio:3mfVersion">1</metadata>
<metadata name="Title">Speckle platter</metadata>
<resources><object id="{OBJ}" p:UUID="{U()}" type="model"><components>{comps}</components></object></resources>
<build p:UUID="{U()}"><item objectid="{OBJ}" p:UUID="{U()}" transform="{tr}" printable="1"/></build>
</model>'''
part_cfg = "".join(f'''<part id="{i}" subtype="normal_part">
<metadata key="name" value="{n}"/><metadata key="matrix" value="{I4}"/><metadata key="extruder" value="{e}"/>
</part>''' for i, (n, _, e) in enumerate(PARTS, 1))
settings = f'''<?xml version="1.0" encoding="UTF-8"?>
<config>
<object id="{OBJ}">
<metadata key="name" value="speckle_platter"/>
<metadata key="extruder" value="{WHITE}"/>
<metadata key="layer_height" value="0.2"/>
<metadata key="enable_support" value="0"/>
{part_cfg}
</object>
<plate>
<metadata key="plater_id" value="1"/><metadata key="plater_name" value=""/><metadata key="locked" value="false"/>
<model_instance><metadata key="object_id" value="{OBJ}"/><metadata key="instance_id" value="0"/><metadata key="identify_id" value="1"/></model_instance>
</plate>
<assemble><assemble_item object_id="{OBJ}" instance_id="0" transform="{tr}" offset="0 0 0"/></assemble>
</config>'''

with zipfile.ZipFile("speckle_platter.3mf", "w", zipfile.ZIP_DEFLATED) as z:
    z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/><Default Extension="config" ContentType="text/xml"/></Types>')
    z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    z.writestr("3D/_rels/3dmodel.model.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/Objects/object_1.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    z.writestr("3D/3dmodel.model", main_model)
    z.writestr("3D/Objects/object_1.model", parts_model)
    z.writestr("Metadata/model_settings.config", settings)

# self-check: every file parses, 3 parts with the right extruders, triangle count intact
with zipfile.ZipFile("speckle_platter.3mf") as z:
    cfg = ET.fromstring(z.read("Metadata/model_settings.config"))
    objs = ET.fromstring(z.read("3D/Objects/object_1.model"))
    ET.fromstring(z.read("3D/3dmodel.model"))
ns = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}
assert len(objs.findall(".//m:triangle", ns)) == n_tris
ext = [p.find("metadata[@key='extruder']").get("value") for p in cfg.iter("part")]
assert ext == [str(e) for _, _, e in PARTS], ext
print(f"speckle_platter.3mf ok: {n_tris} tris, parts -> filaments {ext}")
