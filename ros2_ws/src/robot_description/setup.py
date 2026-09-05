import os
from glob import glob
from setuptools import setup

package_name = 'robot_description'


def data_files_preserving_tree(pattern):
    """Glob recursively and group files by their own directory, so
    subcarpetas como urdf/urdf_der o meshes/meshes_izq se instalan
    conservando su estructura (necesaria para package://<pkg>/... )."""
    grouped = {}
    for path in glob(pattern, recursive=True):
        if os.path.isfile(path):
            grouped.setdefault(os.path.dirname(path), []).append(path)
    return [(os.path.join('share', package_name, d), files)
            for d, files in grouped.items()]


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *data_files_preserving_tree('urdf/**/*.urdf'),
        *data_files_preserving_tree('urdf/**/*.xacro'),
        *data_files_preserving_tree('meshes/**/*.STL'),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david.collazos_her@uao.edu.co',
    description='Descripcion URDF del robot bipedo (pata de 3 DOF)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)
