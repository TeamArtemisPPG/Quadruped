from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'quad'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/quad']),
        ('share/quad', ['package.xml']),

        # ✅ Launch files
        # 🔥 THIS installs ALL launch files
        (os.path.join('share', 'quad', 'launch'),
            glob('launch/*.launch.py')
        ),

        # ✅ URDF files
        ('share/quad/urdf', [
            'urdf/urdf.urdf',
            'urdf/urdf_harmonic.urdf',
            'urdf/urdf.sdf',
            'urdf/model.config'
        ]),

        # ✅ Mesh files (THIS FIXES YOUR ERROR)
        ('share/quad/meshes', [
            'meshes/body-v1.stl',
            'meshes/leg11.stl',
            'meshes/leg12.stl',
            'meshes/leg13.stl',
            'meshes/leg21.stl',
            'meshes/leg22.stl',
            'meshes/leg23.stl',
            'meshes/leg31.stl',
            'meshes/leg32.stl',
            'meshes/leg33.stl',
            'meshes/leg41.stl',
            'meshes/leg42.stl',
            'meshes/leg43.stl',
        ]),
        (os.path.join('share', 'quad', 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', 'quad', 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aanimesh',
    maintainer_email='aanimesh@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_control = quad.ik_control:main',
            'walk_g = quad.walk_gait:main',
            'fwd_ik = quad.fwd:main',
            'gz_ik = quad.gz_ik:main',
            'esp = quad.esp:main',
            'up_go = quad.up_go:main',
            'cal_servo = quad.cal_servo:main',
            'stompping_gait = quad.stompping_gait:main',
            'walk_g_original = quad.walk_gait_original:main',
            'walk_gait_test = quad.walk_gait_test:main',
            'walk_g_original_good = quad.walk_gait_original_good:main',
            'walk_coordinates = quad.walk_coordinates:main',
            'walk_simple = quad.walk_simple:main',
            'crawl_basic = quad.crawl_basic:main',
            'quad_crawl = quad.quad_crawl:main',
            'walk_new= quad.walk_new:main',
        ],
    },
)
