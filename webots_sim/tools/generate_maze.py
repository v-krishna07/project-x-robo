#!/usr/bin/env python3
# ============================================================================
# FILE: tools/generate_maze.py
# Generates worlds/meshmerize.wbt matching Figure 1 of the IIT Bombay
# Techfest Meshmerize Problem Statement.
# ============================================================================

import os

ARENA_SIZE_MM = 2300.0
HALF_ARENA_MM = 1150.0
LINE_WIDTH_MM = 30.0
LINE_THICKNESS_MM = 1.0
END_BOX_SIZE_MM = 400.0


def mm_to_m(val_mm):
    return val_mm / 1000.0


def cad_mm_to_webots_mm(x_cad_mm, y_cad_mm):
    return (x_cad_mm - HALF_ARENA_MM, HALF_ARENA_MM - y_cad_mm)


def format_white_strip_pose(center_x_mm, center_y_mm, size_x_mm, size_y_mm):
    cx = mm_to_m(center_x_mm)
    cy = mm_to_m(center_y_mm)
    cz = mm_to_m(LINE_THICKNESS_MM / 2.0)
    sx = mm_to_m(size_x_mm)
    sy = mm_to_m(size_y_mm)
    sz = mm_to_m(LINE_THICKNESS_MM)
    return (
        f'    Pose {{\n'
        f'      translation {cx:.4f} {cy:.4f} {cz:.4f}\n'
        f'      children [\n'
        f'        Shape {{\n'
        f'          appearance PBRAppearance {{\n'
        f'            baseColor 1 1 1\n'
        f'            roughness 0.8\n'
        f'            metalness 0\n'
        f'          }}\n'
        f'          geometry Box {{\n'
        f'            size {sx:.4f} {sy:.4f} {sz:.4f}\n'
        f'          }}\n'
        f'        }}\n'
        f'      ]\n'
        f'    }}\n'
    )


def get_official_ps_arena_data():
    cad_segments_mm = [
        (1815.0, 2000.0, 2095.0, 2000.0),  # START horizontal stub
        (1815.0, 1604.0, 1815.0, 2230.0),  # Lower-right vertical trunk
        (1515.0, 1604.0, 2115.0, 1604.0),  # Horizontal T-bar at Y=160.4 cm
        (1515.0, 1604.0, 1515.0, 1904.0),  # Vertical dead-end hanging down
        (1815.0, 1200.0, 1815.0, 1604.0),  # Vertical up to Checkpoint 1
        (1515.0, 1200.0, 2115.0, 1200.0),  # Horizontal through Checkpoint 1
        (2115.0,  215.0, 2115.0, 1200.0),  # Far-right vertical through Checkpoint 2
        (1815.0, 1000.0, 2115.0, 1000.0),  # Inner hook bottom horizontal
        (1815.0,  720.0, 1815.0, 1000.0),  # Inner hook vertical
        (1515.0,  720.0, 2115.0,  720.0),  # Horizontal at Y=72 cm
        (1515.0,  215.0, 1515.0, 1000.0),  # Vertical through Checkpoint 3
        ( 215.0,  215.0, 2115.0,  215.0),  # Top horizontal highway
        (1215.0,  215.0, 1215.0,  915.0),  # Center-top vertical drop
        ( 915.0,  215.0,  915.0,  515.0),  # Upper U-loop left vertical
        ( 915.0,  515.0, 1215.0,  515.0),  # Upper U-loop bottom horizontal
        ( 215.0,  215.0,  215.0,  515.0),  # Top-left outer vertical
        ( 215.0,  515.0,  515.0,  515.0),  # Top-left lower horizontal
        ( 515.0,  215.0,  515.0, 1200.0),  # Left vertical trunk through Checkpoint 4
        ( 215.0,  915.0, 1215.0,  915.0),  # Horizontal through Checkpoint 4
        ( 815.0,  915.0,  815.0, 1515.0),  # Lower-middle vertical dead-end
        ( 815.0, 1200.0, 1215.0, 1200.0),  # Lower-middle horizontal spur
        ( 215.0, 1200.0,  515.0, 1200.0),  # Step West to far-left highway
        ( 215.0, 1200.0,  215.0, 2115.0),  # Far-left vertical highway South
        ( 215.0, 2115.0,  515.0, 2115.0),  # Final horizontal entry into END box
    ]

    strips_vrml = []
    for (x1, y1, x2, y2) in cad_segments_mm:
        wx1, wy1 = cad_mm_to_webots_mm(x1, y1)
        wx2, wy2 = cad_mm_to_webots_mm(x2, y2)
        strips_vrml.append(format_white_strip_pose(
            (wx1 + wx2) / 2.0, (wy1 + wy2) / 2.0,
            abs(wx2 - wx1) + LINE_WIDTH_MM, abs(wy2 - wy1) + LINE_WIDTH_MM
        ))

    end_wx, end_wy = cad_mm_to_webots_mm(700.0, 2030.0)
    strips_vrml.append(format_white_strip_pose(end_wx, end_wy, END_BOX_SIZE_MM, END_BOX_SIZE_MM))

    start_wx, start_wy = cad_mm_to_webots_mm(2040.0, 2000.0)
    start_pose = (mm_to_m(start_wx), mm_to_m(start_wy), 0.0210, 3.14159)
    return strips_vrml, start_pose


def main():
    strips_vrml, start_pose = get_official_ps_arena_data()
    bx, by, bz, brot = start_pose
    arena_m = mm_to_m(ARENA_SIZE_MM)
    all_strips = "".join(strips_vrml)

    wbt_text = (
        f'#VRML_SIM R2025a utf8\n\n'
        f'EXTERNPROTO "../protos/LineBot.proto"\n\n'
        f'WorldInfo {{\n'
        f'  info [ "Techfest IIT Bombay Meshmerize Arena" ]\n'
        f'  title "Meshmerize LMS Arena (Official PS Figure 1)"\n'
        f'  basicTimeStep 10\n'
        f'  contactProperties [\n'
        f'    ContactProperties {{ material1 "caster_slip" coulombFriction [ 0.01 ] bounce 0.0 }}\n'
        f'    ContactProperties {{ material1 "wheel_grip"  coulombFriction [ 2.00 ] bounce 0.0 }}\n'
        f'  ]\n'
        f'}}\n'
        f'Viewpoint {{\n'
        f'  orientation 1 0 0 0.7854\n'
        f'  position 0 -2.6 2.6\n'
        f'  near 0.05\n'
        f'}}\n'
        f'Background {{ skyColor [ 0.15 0.18 0.22 ] }}\n'
        f'DirectionalLight {{ ambientIntensity 0.6 direction -0.3 -0.5 -1 intensity 1.8 castShadows TRUE }}\n'
        f'Solid {{\n'
        f'  name "arena_floor"\n'
        f'  children [\n'
        f'    Pose {{\n'
        f'      translation 0 0 -0.005\n'
        f'      children [\n'
        f'        Shape {{\n'
        f'          appearance PBRAppearance {{ baseColor 0.05 0.05 0.05 roughness 0.9 metalness 0 }}\n'
        f'          geometry Box {{ size {arena_m:.3f} {arena_m:.3f} 0.010 }}\n'
        f'        }}\n'
        f'      ]\n'
        f'    }}\n'
        f'{all_strips}'
        f'  ]\n'
        f'  boundingObject Pose {{\n'
        f'    translation 0 0 -0.005\n'
        f'    children [ Box {{ size {arena_m:.3f} {arena_m:.3f} 0.010 }} ]\n'
        f'  }}\n'
        f'}}\n'
        f'LineBot {{\n'
        f'  translation {bx:.4f} {by:.4f} {bz:.4f}\n'
        f'  rotation 0 0 1 {brot:.5f}\n'
        f'  name "MeshmerizeBot"\n'
        f'  controller "line_bot_py"\n'
        f'  supervisor TRUE\n'
        f'}}\n'
    )

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_path = os.path.join(base_dir, "worlds", "meshmerize.wbt")
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    with open(world_path, "w", encoding="utf-8") as wf:
        wf.write(wbt_text)
    print("Generated world:", world_path)


if __name__ == "__main__":
    main()
