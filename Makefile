pc_all:
	colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --build-base build_pc --install-base install_pc  --parallel-workers 20 --event-handlers=console_direct+ --packages-up-to depth2cloud
	cp build_pc/*.json .
pc:
	colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --build-base build_pc --install-base install_pc  --parallel-workers 20 --event-handlers=console_direct+ --packages-select depth2cloud
	cp build_pc/*.json .
pc_debug:
	colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-args -DCMAKE_BUILD_TYPE=Debug --build-base build_pc --install-base install_pc  --parallel-workers 20 --event-handlers=console_direct+ --packages-select depth2cloud
	cp build_pc/*.json .
clean:
	rm -rf build* install* log*
