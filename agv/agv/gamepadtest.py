import pygame
import time

# pygame 초기화
pygame.init()

# 조이스틱 초기화
pygame.joystick.init()

# 첫 번째 조이스틱 객체 가져오기
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("조이스틱이 연결되어 있지 않습니다.")
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

try:
    while True:
        # 이벤트 펌프 실행
        pygame.event.pump()

        if joystick_count > 0:
            # 버튼 입력 읽기
            buttons = joystick.get_numbuttons()
            for i in range(buttons):
                button = joystick.get_button(i)
                if button:
                    print(f"버튼 {i} 눌림")

            # 조이스틱 움직임 읽기
            axes = joystick.get_numaxes()
            labels = ['l_lr, l_ud','l2' ,'r_lr, r_ud', 'r2']

            for i in range(axes):
                axis = joystick.get_axis(i)
                print(f"축 {i}: {round(axis,1)}  | ", end=" ")
                


            print()

            # 딜레이 추가
            time.sleep(0.1)
except KeyboardInterrupt:
    print("프로그램을 종료합니다.")
finally:
    pygame.quit()
