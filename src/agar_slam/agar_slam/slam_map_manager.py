import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from slam_toolbox.srv import SerializePoseGraph

from agar_msgs.srv import ListSlamMaps, SaveSlamMap, SetActiveSlamMap


DEFAULT_MAPS_DIR = '/home/nvidia/workspace/maps'
DEFAULT_ACTIVE_MAP = '/home/nvidia/workspace/maps/active_map.yaml'
DEFAULT_ACTIVE_POSEGRAPH = '/home/nvidia/workspace/maps/active_posegraph.posegraph'


class SlamMapStore:
    def __init__(
        self,
        maps_dir=DEFAULT_MAPS_DIR,
        active_map=DEFAULT_ACTIVE_MAP,
        active_posegraph=DEFAULT_ACTIVE_POSEGRAPH,
        timeout_sec=60.0,
    ):
        self.maps_dir = Path(maps_dir).expanduser()
        self.active_map = Path(active_map).expanduser()
        self.active_posegraph = Path(active_posegraph).expanduser()
        self.timeout_sec = float(timeout_sec)

    def normalize_map_name(self, value):
        text = str(value or '').strip()
        if text.endswith('.yaml'):
            text = text[:-5]
        elif text.endswith('.pgm'):
            text = text[:-4]
        elif text.endswith('.posegraph'):
            text = text[:-10]
        elif text.endswith('.data'):
            text = text[:-5]

        text = os.path.basename(text)
        text = re.sub(r'\s+', '_', text)
        text = re.sub(r'[^A-Za-z0-9_.-]', '_', text)
        text = text.strip('._-')

        if not text:
            raise ValueError('Map name is empty after sanitizing.')
        return text

    def map_paths(self, map_name):
        safe_name = self.normalize_map_name(map_name)
        base_path = self.maps_dir / safe_name
        return safe_name, base_path, base_path.with_suffix('.yaml'), base_path.with_suffix('.pgm')

    def posegraph_paths(self, map_name):
        safe_name = self.normalize_map_name(map_name)
        base_path = self.maps_dir / safe_name
        return (
            safe_name,
            base_path,
            base_path.with_suffix('.posegraph'),
            base_path.with_suffix('.data'),
        )

    def save_map(self, map_name, set_active=True, overwrite=False):
        safe_name, base_path, yaml_path, image_path = self.map_paths(map_name)
        self.maps_dir.mkdir(parents=True, exist_ok=True)

        if not overwrite and (yaml_path.exists() or image_path.exists()):
            return {
                'success': False,
                'message': f'Map already exists: {safe_name}',
                'yaml_path': str(yaml_path),
                'image_path': str(image_path),
                'active_map_path': str(self.active_map),
            }

        cmd = [
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
            str(base_path),
        ]

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.timeout_sec,
            )
        except subprocess.TimeoutExpired:
            return {
                'success': False,
                'message': f'map_saver_cli timed out after {self.timeout_sec:.0f}s',
                'yaml_path': str(yaml_path),
                'image_path': str(image_path),
                'active_map_path': str(self.active_map),
            }
        except Exception as exc:
            return {
                'success': False,
                'message': f'map_saver_cli failed to start: {exc}',
                'yaml_path': str(yaml_path),
                'image_path': str(image_path),
                'active_map_path': str(self.active_map),
            }

        if result.returncode != 0:
            detail = (result.stderr or result.stdout or '').strip()
            if len(detail) > 500:
                detail = detail[-500:]
            return {
                'success': False,
                'message': f'map_saver_cli failed: {detail}',
                'yaml_path': str(yaml_path),
                'image_path': str(image_path),
                'active_map_path': str(self.active_map),
            }

        if not yaml_path.exists() or not image_path.exists():
            return {
                'success': False,
                'message': 'map_saver_cli finished, but yaml/pgm files were not found.',
                'yaml_path': str(yaml_path),
                'image_path': str(image_path),
                'active_map_path': str(self.active_map),
            }

        if set_active:
            active_result = self.set_active_map(str(yaml_path))
            if not active_result['success']:
                return {
                    'success': False,
                    'message': f'Map saved, but active map was not changed: {active_result["message"]}',
                    'yaml_path': str(yaml_path),
                    'image_path': str(image_path),
                    'active_map_path': str(self.active_map),
                }

        return {
            'success': True,
            'message': f'Map saved: {safe_name}',
            'yaml_path': str(yaml_path),
            'image_path': str(image_path),
            'active_map_path': str(self.active_map),
        }

    def resolve_existing_yaml(self, map_name):
        text = str(map_name or '').strip()
        if not text:
            raise ValueError('Map name is empty.')

        candidate = Path(text).expanduser()
        if candidate.is_absolute():
            yaml_path = candidate
        else:
            if text.endswith('.yaml'):
                text = text[:-5]
            safe_name = self.normalize_map_name(text)
            yaml_path = self.maps_dir / f'{safe_name}.yaml'

        maps_root = self.maps_dir.resolve(strict=False)
        target = yaml_path.resolve(strict=False)
        if not target.is_relative_to(maps_root):
            raise ValueError(f'Map must be inside {self.maps_dir}')
        if not yaml_path.exists():
            raise FileNotFoundError(f'Map yaml does not exist: {yaml_path}')
        if yaml_path.suffix != '.yaml':
            raise ValueError('Active map target must be a .yaml file.')
        return yaml_path

    def set_active_map(self, map_name):
        try:
            target = self.resolve_existing_yaml(map_name)
            self.active_map.parent.mkdir(parents=True, exist_ok=True)
            temp_link = self.active_map.with_name(self.active_map.name + '.tmp')
            if temp_link.exists() or temp_link.is_symlink():
                temp_link.unlink()
            os.symlink(str(target), str(temp_link))
            os.replace(str(temp_link), str(self.active_map))
            return {
                'success': True,
                'message': f'Active map set to {target.name}',
                'active_map_path': str(self.active_map),
                'target_map_path': str(target),
            }
        except Exception as exc:
            return {
                'success': False,
                'message': str(exc),
                'active_map_path': str(self.active_map),
                'target_map_path': '',
            }

    def list_maps(self):
        try:
            self.maps_dir.mkdir(parents=True, exist_ok=True)
            map_files = sorted(
                path for path in self.maps_dir.glob('*.yaml')
                if path.name != self.active_map.name
            )
            active_target = ''
            if self.active_map.exists() or self.active_map.is_symlink():
                active_target = os.path.realpath(self.active_map)
            return {
                'success': True,
                'message': f'Found {len(map_files)} map(s).',
                'map_names': [path.stem for path in map_files],
                'yaml_paths': [str(path) for path in map_files],
                'active_map_path': str(self.active_map),
                'active_map_target': active_target,
            }
        except Exception as exc:
            return {
                'success': False,
                'message': str(exc),
                'map_names': [],
                'yaml_paths': [],
                'active_map_path': str(self.active_map),
                'active_map_target': '',
            }

    def set_active_posegraph(self, posegraph_path):
        target = Path(posegraph_path).expanduser()
        data_target = target.with_suffix('.data')
        maps_root = self.maps_dir.resolve(strict=False)
        resolved_target = target.resolve(strict=False)
        resolved_data_target = data_target.resolve(strict=False)
        if not resolved_target.is_relative_to(maps_root):
            raise ValueError(f'Posegraph must be inside {self.maps_dir}')
        if not resolved_data_target.is_relative_to(maps_root):
            raise ValueError(f'Posegraph data must be inside {self.maps_dir}')
        if not target.exists():
            raise FileNotFoundError(f'Posegraph does not exist: {target}')
        if not data_target.exists():
            raise FileNotFoundError(f'Posegraph data does not exist: {data_target}')
        if target.suffix != '.posegraph':
            raise ValueError('Active posegraph target must be a .posegraph file.')

        self.active_posegraph.parent.mkdir(parents=True, exist_ok=True)
        active_data = self.active_posegraph.with_suffix('.data')
        temp_posegraph_link = self.active_posegraph.with_name(
            self.active_posegraph.name + '.tmp'
        )
        temp_data_link = active_data.with_name(active_data.name + '.tmp')
        temp_links = (temp_posegraph_link, temp_data_link)

        try:
            for temp_link in temp_links:
                if temp_link.exists() or temp_link.is_symlink():
                    temp_link.unlink()

            os.symlink(str(resolved_target), str(temp_posegraph_link))
            os.symlink(str(resolved_data_target), str(temp_data_link))

            # The posegraph link is replaced last because it is the entry point
            # supplied to slam_toolbox as a filename without an extension.
            os.replace(str(temp_data_link), str(active_data))
            os.replace(str(temp_posegraph_link), str(self.active_posegraph))
        finally:
            for temp_link in temp_links:
                if temp_link.exists() or temp_link.is_symlink():
                    temp_link.unlink()


class SlamMapManager(Node):
    def __init__(self):
        super().__init__('slam_map_manager')
        self.declare_parameter('maps_dir', DEFAULT_MAPS_DIR)
        self.declare_parameter('active_map', DEFAULT_ACTIVE_MAP)
        self.declare_parameter('map_saver_timeout_sec', 60.0)

        self.store = SlamMapStore(
            maps_dir=self.get_parameter('maps_dir').value,
            active_map=self.get_parameter('active_map').value,
            timeout_sec=self.get_parameter('map_saver_timeout_sec').value,
        )

        self.create_service(SaveSlamMap, 'slam_map_manager/save_map', self.save_map_callback)
        self.create_service(SetActiveSlamMap, 'slam_map_manager/set_active_map', self.set_active_map_callback)
        self.create_service(ListSlamMaps, 'slam_map_manager/list_maps', self.list_maps_callback)
        self.get_logger().info('SLAM map manager ready.')

    def save_map_callback(self, request, response):
        result = self.store.save_map(
            request.map_name,
            set_active=bool(request.set_active),
            overwrite=bool(request.overwrite),
        )
        response.success = result['success']
        response.message = result['message']
        response.yaml_path = result['yaml_path']
        response.image_path = result['image_path']
        response.active_map_path = result['active_map_path']
        self._log_result('save_map', result)
        return response

    def set_active_map_callback(self, request, response):
        result = self.store.set_active_map(request.map_name)
        response.success = result['success']
        response.message = result['message']
        response.active_map_path = result['active_map_path']
        response.target_map_path = result['target_map_path']
        self._log_result('set_active_map', result)
        return response

    def list_maps_callback(self, request, response):
        result = self.store.list_maps()
        response.success = result['success']
        response.message = result['message']
        response.map_names = result['map_names']
        response.yaml_paths = result['yaml_paths']
        response.active_map_path = result['active_map_path']
        response.active_map_target = result['active_map_target']
        return response

    def _log_result(self, action, result):
        if result['success']:
            self.get_logger().info('%s: %s' % (action, result['message']))
        else:
            self.get_logger().error('%s: %s' % (action, result['message']))


def build_store_from_args(args):
    return SlamMapStore(
        maps_dir=args.maps_dir,
        active_map=args.active_map,
        active_posegraph=getattr(args, 'active_posegraph', DEFAULT_ACTIVE_POSEGRAPH),
        timeout_sec=args.timeout,
    )


def save_cli_main(argv=None):
    parser = argparse.ArgumentParser(description='Save the current /map to the AgAR maps directory.')
    parser.add_argument('map_name')
    parser.add_argument('--maps-dir', default=DEFAULT_MAPS_DIR)
    parser.add_argument('--active-map', default=DEFAULT_ACTIVE_MAP)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--no-active', action='store_true')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args(argv)

    result = build_store_from_args(args).save_map(
        args.map_name,
        set_active=not args.no_active,
        overwrite=args.overwrite,
    )
    print(result['message'])
    if result.get('yaml_path'):
        print(result['yaml_path'])
    return 0 if result['success'] else 1


def save_posegraph_cli_main(argv=None):
    parser = argparse.ArgumentParser(description='Serialize the current slam_toolbox posegraph.')
    parser.add_argument('map_name')
    parser.add_argument('--maps-dir', default=DEFAULT_MAPS_DIR)
    parser.add_argument('--active-map', default=DEFAULT_ACTIVE_MAP)
    parser.add_argument('--active-posegraph', default=DEFAULT_ACTIVE_POSEGRAPH)
    parser.add_argument('--timeout', type=float, default=60.0)
    parser.add_argument('--no-active', action='store_true')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args(argv)

    store = build_store_from_args(args)
    safe_name, base_path, posegraph_path, data_path = store.posegraph_paths(
        args.map_name
    )
    store.maps_dir.mkdir(parents=True, exist_ok=True)

    if not args.overwrite and (posegraph_path.exists() or data_path.exists()):
        print(f'Posegraph already exists: {safe_name}')
        print(posegraph_path)
        print(data_path)
        return 1

    rclpy.init(args=None)
    node = Node('save_slam_posegraph')
    try:
        client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=args.timeout):
            print(f'slam_toolbox serialize service not available after {args.timeout:.0f}s')
            return 1

        request = SerializePoseGraph.Request()
        request.filename = str(base_path)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            print(f'slam_toolbox serialize timed out after {args.timeout:.0f}s')
            return 1

        response = future.result()
        if response is None:
            print('slam_toolbox serialize failed: no response')
            return 1
        if int(response.result) != int(SerializePoseGraph.Response.RESULT_SUCCESS):
            print(f'slam_toolbox serialize failed with result {response.result}')
            return 1
        if not posegraph_path.exists():
            print(f'slam_toolbox reported success, but posegraph was not found: {posegraph_path}')
            return 1
        if not data_path.exists():
            print(f'slam_toolbox reported success, but posegraph data was not found: {data_path}')
            return 1

        if not args.no_active:
            store.set_active_posegraph(posegraph_path)

        print(f'Posegraph saved: {safe_name}')
        print(posegraph_path)
        print(data_path)
        if not args.no_active:
            print(f'active posegraph: {store.active_posegraph} -> {posegraph_path}')
            print(
                'active posegraph data: '
                f'{store.active_posegraph.with_suffix(".data")} -> {data_path}'
            )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def set_active_cli_main(argv=None):
    parser = argparse.ArgumentParser(description='Set the active SLAM map symlink.')
    parser.add_argument('map_name')
    parser.add_argument('--maps-dir', default=DEFAULT_MAPS_DIR)
    parser.add_argument('--active-map', default=DEFAULT_ACTIVE_MAP)
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args(argv)

    result = build_store_from_args(args).set_active_map(args.map_name)
    print(result['message'])
    if result.get('target_map_path'):
        print(result['target_map_path'])
    return 0 if result['success'] else 1


def list_cli_main(argv=None):
    parser = argparse.ArgumentParser(description='List saved SLAM maps.')
    parser.add_argument('--maps-dir', default=DEFAULT_MAPS_DIR)
    parser.add_argument('--active-map', default=DEFAULT_ACTIVE_MAP)
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args(argv)

    result = build_store_from_args(args).list_maps()
    print(result['message'])
    if result['active_map_target']:
        print('active: %s -> %s' % (result['active_map_path'], result['active_map_target']))
    else:
        print('active: not set')
    for name, path in zip(result['map_names'], result['yaml_paths']):
        print('%s: %s' % (name, path))
    return 0 if result['success'] else 1


def manager_main():
    rclpy.init()
    node = SlamMapManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    return manager_main()


if __name__ == '__main__':
    sys.exit(manager_main())
