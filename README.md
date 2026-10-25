Nama Lengkap: Muhammad Farrel Rizky Alandra
NRP: 5024261052

# Analisis Singkat & Kendala

## calib.cpp
    Program ini dibuat untuk mendapatkan beberapa informasi untuk kalibrasi kamera dalam bentuk matrix 'P(3x4)', matrix 'K(3x3)', matrix 't(3x1)', dan koefisien distorsi '(1x5)'. Informasi tersebut digunakan untuk mentransformasi sebuah koordinat dalam bentuk pixel menjadi sebuah koordinat dalam dunia nyata relatif terhadap kamera serta mengkalibrasi kamera untuk mencegah distorsi lensa. Program ini dibuat menggunakan OpenCV dengan mendeteksi titik ujung dalam chessboard sebagai referensi. Kemudian, value akan didapatkan dengan menyesuaikan rumus Pinhole Camera Model 'P = K[R|t]'.
    
## main.cpp
    Program ini dibuat untuk mendeteksi objek dengan informasi yang telah didapat dari calib.cpp yang disimpan dalam bentuk file yaml. Sistem deteksi dibuat menggunakan OpenCV dengan segmentasi per objek. Segmentasi dilakukan dengan menggunakan slider sebagai sarana "tuning" objek sehingga mendapatkan value yang diinginkan untuk nantinya digunakan untuk mendeteksi posisi kontur. Kemudian koordinat didapatkan dengan melakukan estimasi jarak untuk setiap objek menggunakan informasi dari file kalibrasi.yaml dimana setiap value disimpan sesuai dengan bentuk umum Pinhole Camera Model.
    
## Kendala
    Hasil thresholding masih belum sesuai dengan keinginan dan kestabilan masih perlu dipertimbangkan kembali. Kedepannya mungkin bisa digunakan metode lain untuk mendapatkan hasil thresholding yang lebih bagus seperti adapative thresholding untuk objek bergerak.
