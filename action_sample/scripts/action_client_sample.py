#!/usr/bin/env python
# -*- coding: utf-8 -*-
#上記2行は必須構文のため、コメント文だと思って削除しないこと
#Python2.7用プログラム

#ROS関係ライブラリ
import rospy #ROSをpythonで使用するのに必要
import actionlib #アクション通信を使用するのに必要
from action_sample.msg import action_fileAction, action_fileGoal #メッセージファイルの読み込み（from パッケージ名.msg import 拡張子なしアクションメッセージファイル名）。actionフォルダで定義した「アクションファイル名.action」ファイルを作成し、catkin_makeすると、「アクションファイル名Action.msg」、「アクションファイル名Feedback.msg」、「アクションファイル名ActionFeedback.msg」、「アクションファイル名Goal.msg」、「アクションファイル名ActionGoal.msg」、「アクションファイル名Result.msg」、「アクションファイル名ActionResult.msg」が生成される。生成されたアクションメッセージファイルは、「ls /home/limlab/catkin_ws/devel/share/パッケージ名/msg」コマンドで確認できる。アクションサーバ側は、「アクションファイル名Action.msg」、「アクションファイル名Result.msg」（途中経過が必要な場合は、「アクションファイル名Feedback.msg」）をインポートする。「アクションファイル名Goal.msg」は、アクションクライアントからリクエストがあった場合に呼び出されるコールバック関数の引数として取得できるため、アクションサーバ側は必要ない。アクションクライアント側は、「アクションファイル名Action.msg」、「アクションファイル名Goal.msg」（途中経過が必要な場合は、「アクションファイル名Feedback.msg」）をインポートする



class Action_Client(): #アクションクライアントのクラス
    def __init__(self):
        self.goal = action_fileGoal() #アクション目標（Goal）のインスタンス生成
        self.action_client = actionlib.SimpleActionClient('action_service_name', action_fileAction) #アクションクライアントのインスタンス生成



    def make_goal(self): #アクション目標（Goal）の作成
        self.goal.text = "アクションリクエスト" #アクション目標（Goal）に代入



    def request_result(self): #アクション結果（Result）のリクエスト
        rospy.loginfo("アクション結果の待機中")
        self.action_client.wait_for_result(rospy.Duration(30)) #結果が返ってくるまで30秒待機。ここで処理が一時停止（ブロッキング）する
        result = self.action_client.get_result() # アクションサーバの「アクションサーバ.set_succeeded(アクション結果)」によって返されたアクション結果を取得
        rospy.loginfo("アクション結果:{}".format(result.action_tf))



    def request_action_service(self): #アクションサービスのリクエスト
        rospy.loginfo("アクションサーバとの接続待機中")
        self.action_client.wait_for_server(rospy.Duration(15)) #アクションサーバーが起動するまで待機。15秒でタイムアウト

        try: #サーバとの接続ができた場合
            self.make_goal() #メッセージの作成
            self.action_client.send_goal(self.goal) #アクションサーバにアクション目標（Goal）を送信。ここでは、定義したアクション目標（Goal）のインスタンスを引数に指定すること。アクション目標（Goal）を送信した時点でアクションサーバが起動（動作開始）し、アクションクライアントも次の処理を実行する（並列処理になる）
            rospy.loginfo("アクション目標の送信:{}".format(self.goal.text)) #ログの表示
            self.request_result() #アクション結果（Result）のリクエスト

        except rospy.ServiceException: #サーバとの接続ができなかった場合
            rospy.loginfo("アクションリクエストに失敗") #ログの表示



def main(): #メイン関数
    rospy.init_node('action_client_sample', anonymous=True) #ノードの初期化と名前の設定
    accli = Action_Client() #クラスのインスタンス作成（クラス内の関数や変数を使えるようにする）
    accli.request_action_service() #アクションサービスのリクエスト
    rospy.spin() #終了防止



if __name__ == "__main__": #Pythonファイル名（__name__）が実行ファイル名（__main__）である場合（このPythonファイルをモジュールとして使用せず、実行ファイルとして扱う場合）
    try: #エラーが発生しなかった場合
        main() #メイン関数の実行
    except rospy.ROSInterruptException: #エラーが発生した場合
        pass #処理の実行をパスする
