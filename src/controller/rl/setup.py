from pathlib import Path

from setuptools import find_packages, setup

package_name = 'rl_controller'
package_root = Path(__file__).parent.resolve()


def collect_data_files() -> list[tuple[str, list[str]]]:
    data_files: list[tuple[str, list[str]]] = [
        (
            'share/ament_index/resource_index/packages',
            [str(package_root / 'resource' / package_name)],
        ),
        (
            f'share/{package_name}',
            [str(package_root / 'package.xml')],
        ),
    ]

    for folder_name in ('config', 'models', 'resources'):
        folder = package_root / folder_name
        if not folder.exists():
            continue
        for file_path in folder.rglob('*'):
            if not file_path.is_file():
                continue
            relative_parent = file_path.parent.relative_to(package_root)
            destination = str(Path('share') / package_name / relative_parent)
            data_files.append((destination, [str(file_path)]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=collect_data_files(),
    install_requires=['setuptools', 'numpy', 'pyyaml', 'pyserial', 'torch', 'mujoco>=3.2.3'],
    zip_safe=True,
    maintainer='RY',
    maintainer_email='18737950912@163.com',
    description='Quadruped RL deployment package for direct Python MuJoCo simulation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
