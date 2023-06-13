from setuptools import setup

package_name = "sensor_bands_gui"

setup(
		name=package_name,
		version="0.0.0",	
		packages=[package_name],
		data_files=[
				("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
		],
		install_requires=["setuptools"],
    zip_safe=True,
		maintainer="inaki",
		maintainer_email="ipujol10@gmail.com",
		description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest", "pynput"],
		entry_points={
				"console_scripts": ["launch_gui = sensor_bands_gui.launch_gui:main"],
		},
)
