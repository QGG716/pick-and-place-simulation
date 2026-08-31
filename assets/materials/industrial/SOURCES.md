# Industrial replay material sources

These files are rendering inputs only. They do not change geometry, collision,
mass, robot dynamics, planning, or qualification thresholds.

## Visual reference brands

- `decals/foton_ollin_logo.png`: downloaded from the [Foton OLLIN official
  website](https://ollin.foton.com.cn/ollin/static/logonew.png). The typical
  visual reference is the OLLIN Suyun box truck, announcement model
  `BJ5048XXY8JEA-AB2`. Its published cargo-body dimensions are not substituted
  for this project's trailer geometry.
- `decals/hxpp_logo.jpg` and `reference/hxpp_company_reference.jpg`: HXPP brand
  reference published in a China Public Relations Association company profile.
  HXPP's [2024 annual report](https://www.hxpp.com.cn/upload/img/photosucai/20250427/%E5%90%88%E5%85%B4%E5%8C%85%E8%A3%852024%E5%B9%B4%E5%B9%B4%E5%BA%A6%E6%8A%A5%E5%91%8A_1745717238752.pdf)
  identifies corrugated cartons and board as its principal products. The logo
  is used as a visual-reference decal, not as a procurement claim.
- FANUC coloration is retained from the exact M-20iD/35 Collada visual assets
  under `assets/robots/fanuc_m20id35`; see that directory's `SOURCE.md`.
- Shanghai Wantai gripper segmentation and appearance are based on the exact
  supplied STEP and installation image under `res/`. No exact public image of
  this custom three-zone assembly was found.

Brand marks remain the property of their respective owners and are included
only for this engineering visualization. Do not redistribute them as generic
texture assets.

## CC0 PBR texture maps

The following 1K JPG maps were downloaded from Poly Haven and are provided
under its [CC0 license](https://polyhaven.com/license):

- `blue_metal_plate/*`: [Blue Metal Plate](https://polyhaven.com/a/blue_metal_plate),
  by Rob Tuytel. Used for coated cargo-body steel.
- `cardboard_box_01/*`: [Cardboard Box 01](https://polyhaven.com/a/cardboard_box_01),
  by Rahul Chaudhary. Retained as a model-specific reference set but not bound
  to the regular simulation cartons because its atlas is not tileable.
- `carton_seamless/carton_512x512.png`: [Seamless Pattern Pack -
  carton](https://opengameart.org/content/seamless-pattern-pack-carton512x512png),
  by n4, CC0. This tileable paper-fibre texture is warm-tinted in the USD
  shader and bound to regular carton visual meshes.
- `rubber_tiles/*`: [Rubber Tiles](https://polyhaven.com/a/rubber_tiles), by
  Amal Kumar. Used for dark matte rubber microtexture on the 72 FG42 visual
  cup lips.

For each asset, `_diff_1k.jpg` is diffuse color, `_nor_gl_1k.jpg` is the
OpenGL normal map, and `_arm_1k.jpg` packs ambient occlusion, roughness, and
metallic channels. The current USD Preview Surface implementation consumes
diffuse and ARM; normal files are retained for a future tangent-basis upgrade.
