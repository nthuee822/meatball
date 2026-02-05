import pygame
import time
import os
import sys

os.environ["SDL_VIDEODRIVER"] = "dummy"

def main():
    try:
        pygame.init()
        pygame.joystick.init()
    except Exception as e:
        print(f"❌ Pygame 初始化失敗: {e}")
        return 1

    if pygame.joystick.get_count() == 0:
        print("❌ 找不到手把，請確認已插上並喚醒")
        return 1
    
    try:
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print(f"✅ 已連接: {joy.get_name()}")
        print(f"📊 總共有 {joy.get_numbuttons()} 顆按鈕")
        print(f"📊 總共有 {joy.get_numaxes()} 個軸")
        print(f"📊 總共有 {joy.get_numhats()} 個方向鍵")
        print("\n👉 請用力壓下左右搖桿 (L3/R3)，看看下面跳出幾號...")
        print("   (按 Ctrl+C 結束)\n")

        try:
            last_buttons = [False] * joy.get_numbuttons()
            
            while True:
                pygame.event.pump()
                
                # 掃描所有按鈕（只在狀態改變時顯示）
                for i in range(joy.get_numbuttons()):
                    pressed = joy.get_button(i)
                    if pressed and not last_buttons[i]:
                        print(f"🔘 偵測到按鈕 ID: {i}")
                    last_buttons[i] = pressed
                
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\n\n✅ 測試結束")
            return 0
            
    except Exception as e:
        print(f"❌ 錯誤: {e}")
        return 1
    finally:
        pygame.quit()

if __name__ == "__main__":
    sys.exit(main())