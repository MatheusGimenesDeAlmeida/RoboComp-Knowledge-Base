from setuptools import find_packages, setup

package_name = 'Exercises'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='borg',
    maintainer_email='matheusga2@al.insper.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quadrado = Exercises.quadrado:main',
            'indeciso = Exercises.indeciso:main',
            'limpador = Exercises.limpador:main',
            'q1_simulado = Exercises.explorador:main'
        ],
    },
)
