import json
import math
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
ROUTE_GRAPH = PACKAGE_DIR / 'params' / 'route_graph.geojson'
STOPS = PACKAGE_DIR / 'params' / 'stops.yaml'
STOP_NODE_TOLERANCE = 1e-6


def load_route_nodes(route_graph_path):
    graph = json.loads(Path(route_graph_path).read_text(encoding='utf-8'))
    nodes = {}
    for feature in graph.get('features', []):
        if feature.get('geometry', {}).get('type') != 'Point':
            continue
        node_id = int(feature.get('properties', {})['id'])
        if node_id in nodes:
            raise ValueError(f'duplicate route node id: {node_id}')
        coords = feature['geometry']['coordinates']
        nodes[node_id] = (float(coords[0]), float(coords[1]))
    return graph, nodes


def validate_edges(graph, nodes):
    edge_ids = set()
    for feature in graph.get('features', []):
        geom_type = feature.get('geometry', {}).get('type')
        if geom_type not in ('LineString', 'MultiLineString'):
            continue
        props = feature.get('properties', {})
        edge_id = int(props['id'])
        if edge_id in edge_ids:
            raise ValueError(f'duplicate route edge id: {edge_id}')
        edge_ids.add(edge_id)

        start_id = int(props['startid'])
        end_id = int(props['endid'])
        if start_id not in nodes:
            raise ValueError(f'edge {edge_id} references missing startid {start_id}')
        if end_id not in nodes:
            raise ValueError(f'edge {edge_id} references missing endid {end_id}')


def validate_stops(stops_path, nodes, tolerance):
    data = yaml.safe_load(Path(stops_path).read_text(encoding='utf-8')) or {}
    stops = data.get('stops')
    if not isinstance(stops, list) or not stops:
        raise ValueError(
            "stops file must contain a non-empty top-level 'stops' list"
        )

    seen = set()
    for stop in stops:
        stop_id = stop['stop_id']
        if stop_id in seen:
            raise ValueError(f'duplicate stop_id: {stop_id}')
        seen.add(stop_id)

        route_node_id = int(stop['route_node_id'])
        if route_node_id not in nodes:
            raise ValueError(
                f'stop {stop_id} references missing '
                f'route_node_id {route_node_id}'
            )

        pose = stop['pose']
        stop_xy = (float(pose['x']), float(pose['y']))
        node_xy = nodes[route_node_id]
        distance = math.hypot(stop_xy[0] - node_xy[0], stop_xy[1] - node_xy[1])
        if distance > tolerance:
            raise ValueError(
                f'stop {stop_id} pose is {distance:.6f} m from '
                f'route node {route_node_id}; '
                f'tolerance is {tolerance:.6f} m'
            )

    return len(stops)


def test_route_graph_and_stops_are_consistent():
    graph, nodes = load_route_nodes(ROUTE_GRAPH)

    assert nodes, 'route graph must contain at least one route node'
    validate_edges(graph, nodes)

    stop_count = validate_stops(STOPS, nodes, STOP_NODE_TOLERANCE)
    assert stop_count > 0, 'stops file must contain at least one stop'
