#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

#ROS関係ライブラリ
import rospy #ROSをpythonで使用するのに必要
import actionlib #アクション通信を使用するのに必要
from action_sample.msg import action_fileAction, action_fileResult #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしアクションメッセージファイル名）。actionフォルダで定義した「アクションファイル名.action」ファイルを作成し、catkin_makeすると、「アクションファイル名Action.msg」、「アクションファイル名Feedback.msg」、「アクションファイル名ActionFeedback.msg」、「アクションファイル名Goal.msg」、「アクションファイル名ActionGoal.msg」、「アクションファイル名Result.msg」、「アクションファイル名ActionResult.msg」が生成される。生成されたアクションメッセージファイルは、「ls /home/limlab/catkin_ws/devel/share/パッケージ名/msg」コマンドで確認できる。アクションサーバ側は、「アクションファイル名Action.msg」、「アクションファイル名Result.msg」（途中経過が必要な場合は、「アクションファイル名Feedback.msg」）をインポートする。「アクションファイル名Goal.msg」は、アクションクライアントからリクエストがあった場合に呼び出されるコールバック関数の引数として取得できるため、アクションサーバ側は必要ない



class Action_Server(): #アクションサーバのクラス
    def __init__(self):
        self.rate = rospy.Rate(0.1) #1秒間に0.1回
        self.result = action_fileResult() #アクション結果（Result）のインスタンス生成
        self.action_server = actionlib.SimpleActionServer('action_service_name', action_fileAction, execute_cb = self.action_callback, auto_start = False) #アクションサーバのインスタンス生成
        self.action_server.start() #アクションサーバーのスタート（サービス通信と違って、自動起動をFalseにした場合はこの開始処理が必要）



    def make_result(self): #アクション結果（Result）の作成
        self.result.action_tf = True #True（ブール値）をアクション結果に代入



    def action_callback(self, goal): #アクションサービスがリクエストされると呼び出されるコールバック関数
        rospy.loginfo("アクション目標の受信：{}".format(goal.text)) #ログの表示
        self.rate.sleep() #待機
        self.make_result() #アクション結果（Result）の作成
        self.action_server.set_succeeded(self.result) #アクション結果をアクションクライアントに返す（アクション結果の送信）。ここでは、定義したアクション結果（Result）のインスタンスを引数に指定すること。アクションクライアント側は、「アクションクライアント名.wait_for_result(タイムアウト時間)」で接続待機し、「result = アクションクライアント名.get_result()」でアクション結果を取得
        rospy.loginfo("アクション結果の送信完了") #ログの表示



def main(): #メイン関数
    rospy.init_node('action_server_sample', anonymous=True) #ノードの初期化と名前の設定
    acsrv = Action_Server() #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    rospy.spin() #終了防止



if __name__ == "__main__": #Pythonファイル名（__name__）が実行ファイル名（__main__）である場合（このPythonファイルをモジュールとして使用せず、実行ファイルとして扱う場合）
    try: #エラーが発生しなかった場合
        main() #メイン関数の実行
    except rospy.ROSInterruptException: #エラーが発生した場合
        pass #処理の実行をパスする
