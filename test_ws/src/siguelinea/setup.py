from setuptools import setup

package_name = 'siguelinea'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jcrex',
    maintainer_email='al428669@uji.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'siguelinea_pub = siguelinea.siguelineas:main',
        	'siguelineas_sub = siguelinea.siguelineas_sub:main',
            'detectar_linea = siguelinea.detectar_linea:main',
        ],
    },
)
