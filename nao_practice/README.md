naoの練習プログラムを置く場所  

なお必要なパッケージはきちんと把握できていない
```nao_move.cpp``` naoの左腕を動かすだけ  
```search.cpp``` 画像処理をする。今は赤い領域の重心を検出する。  
```camera_test.cpp``` naoのカメラから画像を取得してsearch関数で画像処理をする  
```camera_and_head.cpp``` 赤い領域を検出すると視線がその領域を追跡する。  
```move_to_target.l``` hsi_color_filterを用いてボールを検出し、その場所まで移動してキックする。```roseus move_to_target.l```で開始。  
```kick.l``` kickする関数を記述。  
