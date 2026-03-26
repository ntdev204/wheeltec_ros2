from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    robot_ip: str = "192.168.0.100"
    server_port: int = 8000
    zmq_cmd_port: int = 5555
    zmq_telemetry_port: int = 5556
    zmq_camera_port: int = 5557
    camera_topic: str = "/camera/color/image_raw"
    db_path: str = "./robot.db"
    ros_domain_id: int = 0

    class Config:
        env_file = ".env"

settings = Settings()
