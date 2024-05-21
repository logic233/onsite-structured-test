from utils.visualizer import Visualizer
import os
if __name__ == '__main__':
    result_path = None
    save_path = None
    xodr_path = r"D:\project_s\onsite\onsite-structured-test\scenario\fragment\0_76_merge_82\0_76_merge_82.xodr"

    FRAGMENT_FOLDER_PATH = r"D:\project_s\onsite\onsite-structured-test\scenario\fragment"
    REPLAY_FOLDER_PATH = r"D:\project_s\onsite\onsite-structured-test\scenario\replay"
    print(os.listdir(FRAGMENT_FOLDER_PATH))
    vis = Visualizer()
    # vis.replay_result(result_path=result_path, save_path=save_path)
    for task in os.listdir(REPLAY_FOLDER_PATH):
        print(task)
        # vis.show_task(mode='FRAGMENT', task=task)
        vis.show_task(mode='REPLAY', task=task)
    # vis.show_map(xodr_path=xodr_path)