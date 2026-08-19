import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os

def create_warehouse_fleet_map():
    fig, ax = plt.subplots(figsize=(14, 20), dpi=300)
    
    # Background styling
    ax.set_facecolor('#f8fafc')
    fig.patch.set_facecolor('#ffffff')
    
    # Warehouse wall boundaries: x in [-6.99, 6.99], y in [-10.45, 10.45]
    wall_x_min, wall_x_max = -6.99, 6.99
    wall_y_min, wall_y_max = -10.45, 10.45
    
    # Outer warehouse perimeter wall (Heavy Industrial Concrete)
    warehouse_rect = patches.Rectangle(
        (wall_x_min, wall_y_min),
        wall_x_max - wall_x_min,
        wall_y_max - wall_y_min,
        linewidth=4.5, edgecolor='#1e293b', facecolor='#ffffff', zorder=1
    )
    ax.add_patch(warehouse_rect)
    
    # 1-meter floor grid
    for x in np.arange(-6.0, 7.0, 1.0):
        ax.axvline(x=x, color='#e2e8f0', linestyle='--', linewidth=0.7, zorder=2)
    for y in np.arange(-10.0, 11.0, 1.0):
        ax.axhline(y=y, color='#e2e8f0', linestyle='--', linewidth=0.7, zorder=2)
        
    # ------------------ GREEN SAFETY TRAFFIC LANES ------------------
    # North-South Central Fairway (x in [-0.5, 0.5])
    main_ns = patches.Rectangle((-0.5, -9.8), 1.0, 19.6, facecolor='#dcfce7', edgecolor='#86efac', linewidth=1.2, alpha=0.8, zorder=3)
    ax.add_patch(main_ns)
    
    # East-West Cross Lanes
    lane_mid = patches.Rectangle((-6.5, -0.5), 13.0, 1.0, facecolor='#dcfce7', edgecolor='#86efac', linewidth=1.2, alpha=0.8, zorder=3)
    lane_north = patches.Rectangle((-6.5, 4.0), 13.0, 1.0, facecolor='#dcfce7', edgecolor='#86efac', linewidth=1.2, alpha=0.8, zorder=3)
    lane_south = patches.Rectangle((-6.5, -5.0), 13.0, 1.0, facecolor='#dcfce7', edgecolor='#86efac', linewidth=1.2, alpha=0.8, zorder=3)
    lane_north_edge = patches.Rectangle((-6.5, 7.5), 13.0, 0.8, facecolor='#dcfce7', edgecolor='#86efac', linewidth=1.0, alpha=0.6, zorder=3)
    
    ax.add_patch(lane_mid)
    ax.add_patch(lane_north)
    ax.add_patch(lane_south)
    ax.add_patch(lane_north_edge)
    
    # ------------------ 4 YELLOW PACKING BAYS ------------------
    # Bays located at y in [2.8, 4.8], x from -1.8 to 6.2
    bays = [
        (-1.8, 2.8, 1.6, 1.8, 'PACKING BAY 1\n(Staging)'),
        (0.2, 2.8, 1.6, 1.8, 'PACKING BAY 2\n(Buffer)'),
        (2.2, 2.8, 1.6, 1.8, 'PACKING BAY 3\n(Sorting)'),
        (4.2, 2.8, 1.8, 1.8, 'PACKING BAY 4\n(Scout Goal Bay)')
    ]
    for (bx, by, bw, bh, blbl) in bays:
        bay_rect = patches.Rectangle((bx, by), bw, bh, linewidth=2, edgecolor='#ca8a04', facecolor='#fef9c3', linestyle='--', alpha=0.7, zorder=4)
        ax.add_patch(bay_rect)
        ax.text(bx + bw/2, by + 0.15, blbl, fontsize=7, fontweight='bold', color='#854d0e', ha='center', va='bottom', zorder=5)

    # ------------------ SHELVES & RACKS ------------------
    # Western Shelf F (Length ~ 9m): x in [-6.6, -4.9], y in [-5.5, 3.5]
    shelf_f = patches.Rectangle((-6.6, -5.5), 1.7, 9.0, linewidth=2.5, edgecolor='#854d0e', facecolor='#fef08a', zorder=5)
    ax.add_patch(shelf_f)
    ax.text(-5.75, -1.0, 'SHELF F - INDUSTRIAL HIGH-BAY RACK (18m x 2.1m)', fontsize=10, fontweight='bold', color='#713f12',
            rotation=90, va='center', ha='center', zorder=6)
            
    # Eastern D/E Shelf Bank (6 modular units at x = 4.73)
    shelf_y_positions = [
        (0.58, 'RACK E-01 (x=4.73, y=0.58)'),
        (-1.24, 'RACK D-01 (x=4.73, y=-1.24)'),
        (-3.04, 'RACK D-02 (x=4.73, y=-3.04)'),
        (-4.83, 'RACK E-02 (x=4.73, y=-4.83)'),
        (-6.75, 'RACK D-03 (x=4.73, y=-6.75)'),
        (-8.67, 'RACK E-03 (x=4.73, y=-8.67)')
    ]
    for sy, slbl in shelf_y_positions:
        shelf_unit = patches.Rectangle((4.3, sy - 0.5), 1.8, 1.0, linewidth=1.8, edgecolor='#854d0e', facecolor='#fef08a', zorder=5)
        ax.add_patch(shelf_unit)
        ax.text(5.2, sy, slbl, fontsize=7, fontweight='bold', color='#713f12', va='center', ha='center', zorder=6)

    # ------------------ STATIC OBSTACLES & DETAILED CLUTTER ------------------
    # 1. Green Industrial Recycling / Dumpster Bin (TrashCanC_01_002) at (-1.59, 7.72)
    green_bin = patches.Rectangle((-1.59 - 0.74, 7.72 - 0.45), 1.48, 0.91, linewidth=2, edgecolor='#14532d', facecolor='#16a34a', zorder=7)
    ax.add_patch(green_bin)
    ax.text(-1.59, 7.72, '♻ GREEN DUMPSTER\n(TrashCanC_01_002)\n[-1.59, 7.72]', fontsize=7.5, fontweight='bold', color='#ffffff', ha='center', va='center', zorder=8)

    # 2. Large Pallet Box Tower (ClutteringA_01_018) at (-1.49, 5.22)
    pallet_a18 = patches.Rectangle((-1.49 - 1.0, 5.22 - 1.08), 2.00, 2.16, linewidth=2, edgecolor='#7c2d12', facecolor='#ea580c', alpha=0.9, zorder=7)
    ax.add_patch(pallet_a18)
    ax.text(-1.49, 5.22, 'PALLET TOWER\n(ClutteringA_01_018)\n[-1.49, 5.22]', fontsize=7.5, fontweight='bold', color='#ffffff', ha='center', va='center', zorder=8)

    # 3. North-East Pallet Stacks (ClutteringA_01_016 & 017)
    pallet_a17 = patches.Rectangle((3.41 - 1.08, 8.62 - 1.0), 2.16, 2.00, linewidth=2, edgecolor='#7c2d12', facecolor='#ea580c', alpha=0.9, zorder=7)
    pallet_a16 = patches.Rectangle((5.71 - 1.08, 8.62 - 1.0), 2.16, 2.00, linewidth=2, edgecolor='#7c2d12', facecolor='#ea580c', alpha=0.9, zorder=7)
    ax.add_patch(pallet_a17)
    ax.add_patch(pallet_a16)
    ax.text(3.41, 8.62, 'HEAVY PALLET STACK\n(ClutteringA_01_017)\n[3.41, 8.62]', fontsize=7.5, fontweight='bold', color='#ffffff', ha='center', va='center', zorder=8)
    ax.text(5.71, 8.62, 'HEAVY PALLET STACK\n(ClutteringA_01_016)\n[5.71, 8.62]', fontsize=7.5, fontweight='bold', color='#ffffff', ha='center', va='center', zorder=8)

    # 4. Box Clutter in Packing Bay 3 (ClutteringC_01_027) at (3.32, 3.82)
    box_c27 = patches.Rectangle((3.32 - 1.03, 3.82 - 0.88), 2.06, 1.77, linewidth=1.8, edgecolor='#9a3412', facecolor='#fdba74', zorder=7)
    ax.add_patch(box_c27)
    ax.text(3.32, 3.82, 'BOX CLUTTER\n(C-27)\n[3.32, 3.82]', fontsize=7, fontweight='bold', color='#7c2d12', ha='center', va='center', zorder=8)

    # 5. Box Clutter in Packing Bay 4 (ClutteringC_01_028) at (5.54, 3.82)
    box_c28 = patches.Rectangle((5.54 - 1.03, 3.82 - 0.88), 2.06, 1.77, linewidth=1.8, edgecolor='#9a3412', facecolor='#fdba74', zorder=7)
    ax.add_patch(box_c28)
    ax.text(5.54, 3.82, 'BOX CLUTTER\n(C-28)\n[5.54, 3.82]', fontsize=7, fontweight='bold', color='#7c2d12', ha='center', va='center', zorder=8)

    # 6. Carton Stack Near Ramp Eastern Entry (ClutteringC_01_031) at (-1.57, 2.30)
    box_c31 = patches.Rectangle((-1.57 - 0.88, 2.30 - 1.03), 1.77, 2.06, linewidth=1.8, edgecolor='#9a3412', facecolor='#fdba74', zorder=7)
    ax.add_patch(box_c31)
    ax.text(-1.57, 2.30, 'CARTON STACK\n(C-31)\n[-1.57, 2.30]', fontsize=7, fontweight='bold', color='#7c2d12', ha='center', va='center', zorder=8)

    # 7. East Wall Clutter (ClutteringC_01_029 & 030) at y=6.14
    box_c30 = patches.Rectangle((3.24 - 0.88, 6.14 - 1.03), 1.77, 2.06, linewidth=1.5, edgecolor='#9a3412', facecolor='#fed7aa', zorder=7)
    box_c29 = patches.Rectangle((5.38 - 0.88, 6.14 - 1.03), 1.77, 2.06, linewidth=1.5, edgecolor='#9a3412', facecolor='#fed7aa', zorder=7)
    ax.add_patch(box_c30)
    ax.add_patch(box_c29)
    ax.text(3.24, 6.14, 'CLUTTER BOX (C-30)\n[3.24, 6.14]', fontsize=6.5, color='#7c2d12', ha='center', va='center', zorder=8)
    ax.text(5.38, 6.14, 'CLUTTER BOX (C-29)\n[5.38, 6.14]', fontsize=6.5, color='#7c2d12', ha='center', va='center', zorder=8)

    # 8. North-West Clutter (ClutteringC_01_032) at (-1.22, 9.41)
    box_c32 = patches.Rectangle((-1.22 - 1.03, 9.41 - 0.88), 2.06, 1.77, linewidth=1.5, edgecolor='#9a3412', facecolor='#fed7aa', zorder=7)
    ax.add_patch(box_c32)
    ax.text(-1.22, 9.41, 'NW CORNER CLUTTER\n(C-32) [-1.22, 9.41]', fontsize=6.5, color='#7c2d12', ha='center', va='center', zorder=8)

    # 9. Yellow Utility Buckets (Bucket_01_020 & 022) at x=0.43
    bucket_20 = patches.Rectangle((0.43 - 0.61, 9.63 - 0.47), 1.22, 0.94, linewidth=1.5, edgecolor='#ca8a04', facecolor='#facc15', zorder=7)
    bucket_22 = patches.Rectangle((0.43 - 0.61, 8.59 - 0.47), 1.22, 0.94, linewidth=1.5, edgecolor='#ca8a04', facecolor='#facc15', zorder=7)
    ax.add_patch(bucket_20)
    ax.add_patch(bucket_22)
    ax.text(0.43, 9.63, 'BUCKETS (20)', fontsize=6.5, fontweight='bold', color='#713f12', ha='center', va='center', zorder=8)
    ax.text(0.43, 8.59, 'BUCKETS (22)', fontsize=6.5, fontweight='bold', color='#713f12', ha='center', va='center', zorder=8)

    # 10. South Staging Clutter (ClutteringD_01_005) at (-1.63, -7.81)
    box_d05 = patches.Rectangle((-1.63 - 0.51, -7.81 - 0.75), 1.02, 1.49, linewidth=1.5, edgecolor='#9a3412', facecolor='#fdba74', zorder=7)
    ax.add_patch(box_d05)
    ax.text(-1.63, -7.81, 'SOUTH CLUTTER\n(D-05) [-1.63, -7.81]', fontsize=6.5, color='#7c2d12', ha='center', va='center', zorder=8)

    # ------------------ CUSTOM 10° ELEVATED RAMP ------------------
    # Ramp Dimensions: Width = 1.5m (x: -4.15 to -2.65), Center x = -3.4
    # South Incline (y: -4.0 to -1.0), Flat Deck (y: -1.0 to 1.0), North Incline (y: 1.0 to 4.0)
    
    # 1. South Ramp Incline
    south_incline = patches.Rectangle((-4.15, -4.0), 1.5, 3.0, linewidth=2, edgecolor='#b91c1c', facecolor='#fca5a5', alpha=0.9, zorder=6)
    ax.add_patch(south_incline)
    for ay in np.arange(-3.5, -1.0, 0.8):
        ax.annotate('', xy=(-3.4, ay + 0.4), xytext=(-3.4, ay),
                    arrowprops=dict(arrowstyle='->', lw=1.8, color='#991b1b'), zorder=7)
    ax.text(-3.4, -2.5, 'SOUTH INCLINE\n(+10.0° / +17.6% Grade)\n[y: -4.0 to -1.0]', fontsize=8, fontweight='bold', color='#7f1d1d', ha='center', va='center', zorder=8)

    # 2. Elevated Platform (z = 0.53m)
    platform = patches.Rectangle((-4.15, -1.0), 1.5, 2.0, linewidth=2.5, edgecolor='#1e3a8a', facecolor='#93c5fd', alpha=0.95, zorder=6)
    ax.add_patch(platform)
    ax.text(-3.4, 0.0, 'ELEVATED\nPLATFORM DECK\n(z = +0.529m)\n[y: -1.0 to +1.0]', fontsize=8.5, fontweight='bold', color='#1e3a8a', ha='center', va='center', zorder=8)

    # 3. North Ramp Decline
    north_incline = patches.Rectangle((-4.15, 1.0), 1.5, 3.0, linewidth=2, edgecolor='#b91c1c', facecolor='#fca5a5', alpha=0.9, zorder=6)
    ax.add_patch(north_incline)
    for ay in np.arange(1.2, 3.5, 0.8):
        ax.annotate('', xy=(-3.4, ay + 0.4), xytext=(-3.4, ay),
                    arrowprops=dict(arrowstyle='->', lw=1.8, color='#991b1b'), zorder=7)
    ax.text(-3.4, 2.5, 'NORTH DECLINE\n(-10.0° / -17.6% Grade)\n[y: +1.0 to +4.0]', fontsize=8, fontweight='bold', color='#7f1d1d', ha='center', va='center', zorder=8)

    # Ramp Outline
    ramp_border = patches.Rectangle((-4.15, -4.0), 1.5, 8.0, linewidth=3, edgecolor='#0f172a', facecolor='none', zorder=9)
    ax.add_patch(ramp_border)

    # ------------------ ROBOT SPAWNS & MISSIONS ------------------
    # AMR-1 Spawn: (0.0, 0.0)
    amr1_spawn = patches.Circle((0.0, 0.0), 0.40, linewidth=3, edgecolor='#1e40af', facecolor='#3b82f6', zorder=10)
    ax.add_patch(amr1_spawn)
    ax.annotate('', xy=(0.0, 0.7), xytext=(0.0, 0.0),
                arrowprops=dict(arrowstyle='->', lw=3, color='#ffffff'), zorder=11)
    ax.text(0.0, -0.75, 'AMR-1 SPAWN\n(0.0, 0.0)\n[Lead / Mapper / Heavy]', fontsize=9, fontweight='bold', color='#1e40af', ha='center', va='top', zorder=12)

    # AMR-2 Spawn: (2.0, 0.0)
    amr2_spawn = patches.Circle((2.0, 0.0), 0.40, linewidth=3, edgecolor='#047857', facecolor='#10b981', zorder=10)
    ax.add_patch(amr2_spawn)
    ax.annotate('', xy=(2.0, 0.7), xytext=(2.0, 0.0),
                arrowprops=dict(arrowstyle='->', lw=3, color='#ffffff'), zorder=11)
    ax.text(2.0, -0.75, 'AMR-2 SPAWN\n(2.0, 0.0)\n[Fast Scout / 1.2 m/s]', fontsize=9, fontweight='bold', color='#047857', ha='center', va='top', zorder=12)

    # ------------------ VERIFIED GOAL POSITIONS (OPEN SOUTHERN QUADRANT) ------------------
    # AMR-1 Goal: SOUTH LOGISTICS BAY at (-2.0, -5.0)
    ax.plot(-2.0, -5.0, marker='*', markersize=26, color='#3b82f6', markeredgecolor='#1e3a8a', markeredgewidth=2.5, zorder=13)
    ax.text(-2.0, -5.7, 'AMR-1 GOAL\nSOUTH LOGISTICS BAY\n(-2.0, -5.0)', fontsize=9, fontweight='bold', color='#1e3a8a', ha='center', va='top', zorder=14)

    # AMR-2 Goal: SOUTH STAGING BAY at (2.0, -5.0)
    ax.plot(2.0, -5.0, marker='*', markersize=26, color='#10b981', markeredgecolor='#065f46', markeredgewidth=2.5, zorder=13)
    ax.text(2.0, -5.7, 'AMR-2 GOAL\nSOUTH STAGING BAY\n(2.0, -5.0)', fontsize=9, fontweight='bold', color='#065f46', ha='center', va='top', zorder=14)

    # Alternate North Waypoints (Reference)
    ax.plot(2.5, 4.5, marker='.', markersize=14, color='#10b981', zorder=13)
    ax.text(2.5, 4.8, 'North Bay 4\n(2.5, 4.5)', fontsize=7, color='#065f46', ha='center', va='bottom', zorder=14)
    ax.plot(-2.0, 4.8, marker='.', markersize=14, color='#3b82f6', zorder=13)
    ax.text(-2.0, 4.8, 'North Heavy\n(-2.0, 4.8)', fontsize=7, color='#1e3a8a', ha='center', va='bottom', zorder=14)

    # ------------------ CONCURRENT AUTONOMOUS MPPI PATHS ------------------
    # AMR-2 MPPI Path (South): (2.0, 0.0) -> (2.0, -2.5) -> (2.0, -5.0)
    amr2_path_x = [2.0, 2.0, 2.0]
    amr2_path_y = [0.0, -2.5, -5.0]
    ax.plot(amr2_path_x, amr2_path_y, color='#059669', linestyle='--', linewidth=4, alpha=0.9, label='AMR-2 MPPI South Trajectory -> (2.0, -5.0) [WIDE OPEN CLEARANCE]', zorder=11)

    # AMR-1 MPPI Path (South): (0.0, 0.0) -> (-0.8, -2.5) -> (-2.0, -5.0)
    amr1_path_x = [0.0, -0.8, -2.0]
    amr1_path_y = [0.0, -2.5, -5.0]
    ax.plot(amr1_path_x, amr1_path_y, color='#2563eb', linestyle='--', linewidth=4, alpha=0.9, label='AMR-1 MPPI South Trajectory -> (-2.0, -5.0) [WIDE OPEN CLEARANCE]', zorder=11)

    # AMR-1 Ramp Shortcut Route: (-2.0, -5.0) -> (-3.4, -4.5) -> (-3.4, 0.0) -> (-3.4, 4.5)
    amr1_ramp_x = [-2.0, -3.4, -3.4, -3.4, -3.4]
    amr1_ramp_y = [-5.0, -4.5, 0.0, 4.0, 4.5]
    ax.plot(amr1_ramp_x, amr1_ramp_y, color='#7c3aed', linestyle=':', linewidth=3.5, alpha=0.85, label='AMR-1 Ramp Physical Shortcut (Cost-Aware Traversal)', zorder=11)

    # ------------------ TITLE & METADATA ------------------
    ax.set_title('AUTONOMOUS HETEROGENEOUS AMR FLEET - 2D WAREHOUSE BLUEPRINT MAP\nFull Environment Inventory: small_warehouse.sdf with 10° Traversable Ramp & Static Clutter',
                 fontsize=13, fontweight='bold', pad=20, color='#0f172a')
    
    ax.set_xlabel('X Coordinate (Meters) [East +]', fontsize=11, fontweight='bold', labelpad=10)
    ax.set_ylabel('Y Coordinate (Meters) [North +]', fontsize=11, fontweight='bold', labelpad=10)
    
    ax.set_xlim(-7.5, 7.5)
    ax.set_ylim(-11.0, 11.5)
    ax.set_aspect('equal')
    
    # Legend
    legend = ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.07), ncol=1, frameon=True, fontsize=9.5)
    legend.get_frame().set_facecolor('#ffffff')
    legend.get_frame().set_edgecolor('#94a3b8')
    legend.get_frame().set_linewidth(1.5)
    
    plt.tight_layout()
    
    # Save files
    os.makedirs('/home/abhinash/AMR_ws/docs', exist_ok=True)
    png_path = '/home/abhinash/AMR_ws/docs/warehouse_fleet_map.png'
    svg_path = '/home/abhinash/AMR_ws/docs/warehouse_fleet_map.svg'
    
    plt.savefig(png_path, dpi=300, bbox_inches='tight')
    plt.savefig(svg_path, bbox_inches='tight')
    plt.close()
    
    print(f"Successfully generated {png_path} and {svg_path}")

if __name__ == '__main__':
    create_warehouse_fleet_map()
