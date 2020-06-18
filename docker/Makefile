.PHONY: build_app

build_app:
		$(info Make: Building Robot Kinematics)
		@DOCKER_BUILDKIT=1 docker build -o ../workspace -f app/Dockerfile ..

build_image:
		$(info Make: Building Robot Kinematics image)
		@docker build -t robot-kinematics:0.5 -f image/Dockerfile ..

build_remote_env:
		@docker build -t rk_remote:0.5 -f remote/Dockerfile ..

build_novnc:
		@docker build -t rk_novnc:0.5 -f novnc/Dockerfile ..

build:
		@make -s build_app
		@make -s build_image

run_backend_linux:
		@docker-compose -f docker-compose-linux.yml up -d

run_backend_cross_platform:
		@docker-compose -f docker-compose-cross-platform.yml up -d

run_frontend:
		@docker run --network="docker_default" -e ROS_MASTER_URI=http://ros-master:11311 -i -t robot-kinematics:0.5 \
		rosrun tui tui /workspace/src/tui/config/config.ini

kill_backend_linux:
		@docker-compose -f docker-compose-linux.yml down

kill_backend_cross_platform:
		@docker-compose -f docker-compose-cross-platform.yml down

see_log:
		@docker logs -t -f rk_application
