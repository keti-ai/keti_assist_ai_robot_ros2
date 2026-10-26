#!/usr/bin/env python3
"""
xacro -> URDF -> MJCF 변환 스크립트

ROS2 workspace가 source 된 환경(WSL2 / Docker)에서 한 번 실행해서
ROS 의존성이 없는 MJCF + 메시 폴더를 만들어 낸다.
이후 MuJoCo 쪽(Windows 네이티브 포함)은 결과 폴더만 있으면 된다.

필요 패키지 (ROS2 환경 안에서):
    pip install mujoco trimesh pycollada

사용 예:
    source ~/ros2_ws/install/setup.bash
    python3 urdf_to_mjcf.py \
        $(ros2 pkg prefix --share my_robot_description)/urdf/robot.urdf.xacro \
        -o ./mjcf_out \
        --xacro-arg use_gripper:=true
"""
import argparse
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

# MuJoCo가 직접 읽을 수 있는 메시 포맷
MJ_MESH_EXT = {".stl", ".obj", ".msh"}
# MuJoCo 변환에 필요 없는 ROS/Gazebo 전용 태그
STRIP_TAGS = {"gazebo", "ros2_control", "transmission"}


def run_xacro(xacro_file: Path, xacro_args: list[str]) -> str:
    cmd = ["xacro", str(xacro_file), *xacro_args]
    return subprocess.check_output(cmd, text=True)


def resolve_uri(uri: str) -> tuple[Path, str]:
    """package:// / file:// / 일반 경로를 실제 경로와 고유 이름으로 바꾼다."""
    if uri.startswith("package://"):
        pkg, rel = uri[len("package://"):].split("/", 1)
        path = Path(get_package_share_directory(pkg)) / rel
        unique = f"{pkg}__{rel}"
    elif uri.startswith("file://"):
        path = Path(uri[len("file://"):])
        unique = path.name
    else:
        path = Path(uri)
        unique = path.name
    # MuJoCo는 URDF 메시 경로의 디렉터리를 버리므로(strippath) 이름을 평탄화
    unique = unique.replace("/", "__").replace("\\", "__")
    return path, unique


def convert_mesh(src: Path, dst_dir: Path, unique: str) -> str:
    """메시를 dst_dir로 복사하고, 필요하면 OBJ로 변환한다. 새 파일명을 반환."""
    ext = src.suffix.lower()
    if ext in MJ_MESH_EXT:
        dst = dst_dir / unique
        if not dst.exists():
            shutil.copy2(src, dst)
        return dst.name

    # DAE 등은 trimesh로 OBJ 변환 (색상/텍스처는 버려짐)
    import trimesh

    dst = dst_dir / (Path(unique).stem + ".obj")
    if not dst.exists():
        mesh = trimesh.load(src, force="mesh")
        mesh.export(dst)
        print(f"  [convert] {src.name} -> {dst.name}")
    return dst.name


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("xacro", type=Path, help="xacro 또는 urdf 파일")
    ap.add_argument("-o", "--out", type=Path, default=Path("mjcf_out"))
    ap.add_argument("--xacro-arg", action="append", default=[],
                    help="xacro 인자 (name:=value), 여러 번 지정 가능")
    ap.add_argument("--keep-visual", action="store_true", default=True,
                    help="visual 메시 유지 (기본 on)")
    ap.add_argument("--min-mass", type=float, default=1e-3,
                    help="질량 없는 링크에 넣을 질량 [kg] (기본 1e-3)")
    ap.add_argument("--min-inertia", type=float, default=1e-6,
                    help="질량 없는 링크에 넣을 관성 [kg·m²] (기본 1e-6)")
    ap.add_argument("--fusestatic", action="store_true",
                    help="fixed joint로 붙은 링크를 부모 body에 병합")
    args = ap.parse_args()

    out = args.out.resolve()
    mesh_dir = out / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)

    # 1) xacro 전개
    urdf_text = run_xacro(args.xacro, args.xacro_arg)
    root = ET.fromstring(urdf_text)

    # 2) ROS/Gazebo 전용 태그 제거
    for tag in STRIP_TAGS:
        for el in root.findall(tag):
            root.remove(el)

    # 2-1) inertial이 없거나 질량이 0인 링크(가상 링크, TCP 프레임 등)에 최소 질량 부여
    #      MuJoCo는 움직이는 body의 mass/inertia가 mjMINVAL 미만이면 컴파일 에러
    patched = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        mass_el = inertial.find("mass") if inertial is not None else None
        mass = float(mass_el.get("value", 0)) if mass_el is not None else 0.0
        if inertial is not None and mass > args.min_mass:
            continue
        if inertial is not None:
            link.remove(inertial)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=f"{args.min_mass:g}")
        ET.SubElement(inertial, "inertia",
                      ixx=f"{args.min_inertia:g}", iyy=f"{args.min_inertia:g}",
                      izz=f"{args.min_inertia:g}", ixy="0", ixz="0", iyz="0")
        patched.append(link.get("name"))
    if patched:
        print(f"[fix] 질량 없는 링크 {len(patched)}개에 최소 질량 부여: {', '.join(patched)}")

    # 3) 메시 경로 해석 + 복사/변환
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename")
        if not uri:
            continue
        src, unique = resolve_uri(uri)
        if not src.exists():
            print(f"  [warn] 메시 없음: {uri} -> {src}", file=sys.stderr)
            continue
        mesh.set("filename", convert_mesh(src, mesh_dir, unique))

    # 4) MuJoCo 컴파일러 옵션 삽입
    for el in root.findall("mujoco"):
        root.remove(el)
    mj = ET.SubElement(root, "mujoco")
    ET.SubElement(mj, "compiler",
                  meshdir="meshes",
                  balanceinertia="true",
                  discardvisual="false" if args.keep_visual else "true",
                  fusestatic="true" if args.fusestatic else "false")

    urdf_out = out / "robot.urdf"
    ET.ElementTree(root).write(urdf_out, encoding="utf-8", xml_declaration=True)
    print(f"[ok] URDF 저장: {urdf_out}")

    # 5) MuJoCo로 컴파일 후 MJCF 저장
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(urdf_out))
    mjcf_out = out / "robot.xml"
    mujoco.mj_saveLastXML(str(mjcf_out), model)
    print(f"[ok] MJCF 저장: {mjcf_out}")
    print(f"     bodies={model.nbody} joints={model.njnt} geoms={model.ngeom} "
          f"meshes={model.nmesh} actuators={model.nu}")
    if model.nu == 0:
        print("     -> 액추에이터가 없음. robot.xml에 <actuator>를 직접 추가해야 함")
    return 0


if __name__ == "__main__":
    sys.exit(main())