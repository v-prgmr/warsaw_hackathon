"""Pure helpers for the AR skeleton: URDF link tree -> limb chains -> polyline points.

No ROS here (like ros_util), so it is unit-tested without a ROS install. skeleton_node.py wraps
these with the /tf lookups and the marker publisher.

The body is drawn as a few LINE_STRIP "strokes", one per limb (root -> leaf), instead of ~30
separate segments: the AR bridge sends one draw_world_annotation per annotation, and a LINE_STRIP
is a single annotation, so this collapses the per-tick Lens traffic from one call per bone to one
per limb. Chains that share a prefix (e.g. both legs share the pelvis) redraw it, which is fine.
"""
import xml.etree.ElementTree as ET


def parse_bones(urdf_xml):
    """URDF string -> list of (parent_link, child_link) from every <joint>, in document order."""
    root = ET.fromstring(urdf_xml)
    bones = []
    for joint in root.iter("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            continue
        p, c = parent.get("link"), child.get("link")
        if p and c:
            bones.append((p, c))
    return bones


def chains(bones):
    """Root-to-leaf link paths (one per leaf) covering every bone; each limb is one stroke.

    ``bones`` is (parent, child) edges of a tree/forest. A leaf is a link that is never a parent;
    a root is a link that is never a child. Paths keep first-seen leaf order for stable marker ids.
    """
    children, parent, is_child = {}, {}, set()
    for p, c in bones:
        children.setdefault(p, []).append(c)
        parent[c] = p
        is_child.add(c)
    leaves, seen = [], set()
    for p, c in bones:
        for link in (p, c):
            if link not in children and link not in seen:
                leaves.append(link)
                seen.add(link)
    paths = []
    for leaf in leaves:
        path, node, guard = [leaf], leaf, 0
        while node in parent and guard < len(parent) + 1:
            node = parent[node]
            path.append(node)
            guard += 1
        paths.append(list(reversed(path)))
    return paths


def chain_point_lists(bones, positions):
    """Per-limb polylines (each a list of >= 2 (x, y, z)) for chains, from resolved link positions.

    ``positions`` maps link name -> (x, y, z). A link with no position breaks its chain, so a
    momentarily missing frame never draws a segment jumping across it; runs shorter than two
    points are dropped.
    """
    out = []
    for chain in chains(bones):
        run = []
        for link in chain:
            p = positions.get(link)
            if p is None:
                if len(run) >= 2:
                    out.append(run)
                run = []
                continue
            if run and run[-1] == p:
                continue
            run.append(p)
        if len(run) >= 2:
            out.append(run)
    return out
