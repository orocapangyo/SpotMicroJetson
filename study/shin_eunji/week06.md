/home/ehdtod001009/micro_spot/SpotMicroJetson/Simulation/pybullet_matlab.py 파일 생성

## **PyBullet에서 궤적 재현**

```python
from matlab_circle import TrajectoryGenerator
from matlab_kinematics import LegIK
```

/home/ehdtod001009/micro_spot/SpotMicroJetson/Simulation/pybullet_matlab.py 실행

**문제1:**

raise ValueError(f"Target ({x}, {y}, {z}) is unreachable. H={H:.2f}")

ValueError: Target (101.91691505040485, -119.90792396734354, -152.86213493064872) is unreachable. H=200.16

로봇 몸체에서 발끝 목표 지점까지의 직선 거리 `H:200.16`> 로봇의 허벅지 `l3:100(mm)`+ 종아리 길이 
`l4: 100(mm)`

해결방법  : radius나 center 감소 or z조절

**문제2:**

작동은 되나 움직임이 없다.

해결방법: 조인트가 맞는가??

1. 조인트 확인

```
    #조인트 정보 얻어오기 
    number_of_joints = p.getNumJoints(robot)
    for joint_number in range(number_of_joints):
        # This prints out all the information returned by getJointInfo().
        # Try it out
        info = p.getJointInfo(robot, joint_number)
        print(info[0], ": ", info[1])
```

```
b3Printf: rear_right_leg_link_cover
0 :  b'base_rear'
1 :  b'base_front'
2 :  b'front_left_shoulder'
3 :  b'front_left_leg'
4 :  b'front_left_leg_cover_joint'
5 :  b'front_left_foot'
6 :  b'front_left_toe'
7 :  b'front_right_shoulder'
8 :  b'front_right_leg'
9 :  b'front_right_leg_cover_joint'
10 :  b'front_right_foot'
11 :  b'front_right_toe'
12 :  b'rear_left_shoulder'
13 :  b'rear_left_leg'
14 :  b'rear_left_leg_cover_joint'
15 :  b'rear_left_foot'
16 :  b'rear_left_toe'
17 :  b'rear_right_shoulder'
18 :  b'rear_right_leg'
19 :  b'rear_right_leg_cover_joint'
20 :  b'rear_right_foot'
21 :  b'rear_right_toe'
```

 # shoulder, hip, knee  순으로 다시 맞게 넣기

```python
        # 관절 인덱스 (URDF 구조에 따라 조정 필요)
        self.joint_indices = {
            'front_left': [2, 3, 5],     # shoulder(2), leg(3), foot(5)
            'front_right': [7, 8, 10],   # shoulder(7), leg(8), foot(10)
            'rear_left': [12, 13, 15],   # shoulder(12), leg(13), foot(15)
            'rear_right': [17, 18, 20]   # shoulder(17), leg(18), foot(20)
        }

```

**문제3:**

나머지 다리를 이상하게 고정했는가?

controller.set_leg_position(leg_name, (100, -100 if 'right' in leg_name else 100, -100))

```python
   앉아있는 자세 참고 
    Lp=np.array([[100,-100,100,1], \
                [100,-100,-100,1], \
                [-100,-100,100,1], \
                [-100,-100,-100,1]])
```

나머지 다리 고정 , 한개의 다리 원형으로 움직이기
