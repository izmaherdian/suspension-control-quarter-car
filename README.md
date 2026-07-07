# Active Suspension Control of a Quarter Car using GA-Optimized LQR-PID

Repositori ini berisi proyek simulasi dan perancangan kontroler **suspensi aktif quarter car (2-DOF)**. Sistem ini dirancang untuk mereduksi getaran pada kendaraan akibat profil jalan yang tidak rata, sehingga dapat meningkatkan kenyamanan berkendara (*ride comfort*) sekaligus menjaga stabilitas kendaraan (*road holding*).

Perancangan kontroler memadukan metode **Linear Quadratic Regulator (LQR)** dan **Proportional-Integral-Derivative (PID)** dengan teknik transformasi berbasis **Backstepping Integral**. Parameter bobot LQR dioptimalkan secara otomatis menggunakan **Algoritma Genetika (Genetic Algorithm - GA)**.

---

## 📌 Deskripsi Sistem dan Parameter

Sistem dimodelkan menggunakan model *quarter car* standar dengan parameter roda depan kendaraan BMW:

| Parameter | Simbol | Nilai | Satuan | Deskripsi |
| :--- | :--- | :--- | :--- | :--- |
| **Tire Stiffness** | $kk$ | $340$ | kN/m | Konstanta pegas ban |
| **Suspension Stiffness** | $kr$ | $30$ | kN/m | Konstanta pegas suspensi |
| **Suspension Damping** | $br$ | $1450$ | Ns/m | Koefisien redaman suspensi |
| **Sprung Mass** | $mc$ | $408$ | kg | Massa bodi/chassis kendaraan |
| **Unsprung Mass** | $mus$ | $48.3$ | kg | Massa roda dan aksel roda |

### Representasi State-Space
Model ruang keadaan (state-space) didefinisikan sebagai:
$$\dot{x} = A x + B u + B_d z_r$$

Dengan status/state variabel:
*   $x_1$: Perpindahan roda/unsprung mass ($z_{us}$)
*   $x_2$: Kecepatan roda/unsprung mass ($\dot{z}_{us}$)
*   $x_3$: Perpindahan bodi mobil/sprung mass ($z_c$)
*   $x_4$: Kecepatan bodi mobil/sprung mass ($\dot{z}_c$)

Input sistem:
*   $u$: Gaya kontrol aktif dari aktuator suspensi.
*   $z_r$: Gangguan perpindahan vertikal jalan.

---

## 🛠️ Struktur Kontrol LQR-PID dengan Backstepping Integral

1.  **Augmentasi Sistem**: Sistem di-augmentasikan agar mampu menampung struktur integrator untuk kontrol PID. Matriks state-space diperluas dari 4-state menjadi 6-state ($A_{aug}$ dan $B_{aug}$).
2.  **Transformasi Gain LQR ke PID**: Matriks gain LQR ($K$) ditransformasikan ke bentuk gain PID ($K_p, K_i, K_d$) melalui relasi transformasi matriks:
    $$K_{hat} = K \cdot \gamma^{\dagger}$$
    Di mana $\gamma$ dibentuk dari matriks output $C$ dan sistem $A, B$, sedangkan $\gamma^{\dagger}$ adalah pseudoinverse dari $\gamma$.
3.  **Optimasi Algoritma Genetika (GA)**: GA digunakan untuk mencari bobot optimal pada matriks diagonal $Q$ dan $R$ dengan meminimalkan fungsi objektif yang menggabungkan:
    *   **Integral Square Error (ISE)** dari deviasi status.
    *   Penalti terhadap **Overshoot** respon suspensi.
    *   Penalti terhadap **Settling Time** (waktu pemulihan getaran).
    *   Penalti terhadap **Steady-state Error**.

---

## 📂 Struktur File Proyek

*   **`CariLQR_PID.slx`**: File model Simulink yang merepresentasikan model dinamika quarter car lengkap dengan loop kontrol aktif dan gangguan jalan.
*   **`FindLQR_PID.m`**: Script utama MATLAB yang melakukan optimasi GA untuk LQR teraugmentasi, memanggil model Simulink `CariLQR_PID.slx` secara otomatis untuk simulasi dinamis, dan menampilkan visualisasi perbandingan respon kontroler baseline vs optimal.
*   **`GA.m`**: Script MATLAB mandiri untuk menguji optimasi GA pada model state-space 4-state standar (tanpa Simulink) dan memvalidasi respon transien serta kestabilan nilai eigen.
*   **`Report.pdf`**: Laporan komprehensif mengenai hasil perancangan dan analisis teoritis dari sistem kontrol ini.

---

## 🚀 Panduan Menjalankan Simulasi (Step-by-Step)

Untuk menjalankan proyek ini di MATLAB, ikuti langkah-langkah berikut:

### 1. Prasyarat Sistem
Pastikan MATLAB Anda sudah terinstal beberapa toolbox berikut:
*   **Control System Toolbox** (untuk fungsi `lqr`, `state-space`, dll.)
*   **Global Optimization Toolbox** (untuk fungsi `ga` dan `optimoptions`)
*   **Simulink** (untuk membuka dan mensimulasikan `.slx`)

### 2. Langkah Simulasi LQR-PID (Menggunakan Simulink)
1.  Buka MATLAB dan arahkan *Current Folder* ke direktori proyek ini.
2.  Jalankan script utama dengan mengetik perintah berikut pada Command Window:
    ```matlab
    run('FindLQR_PID.m')
    ```
3.  Proses optimasi menggunakan Algoritma Genetika akan berjalan selama beberapa generasi (default: maksimal 1000 generasi dengan populasi 20). Perkembangan nilai biaya (*cost*) akan ditampilkan secara iteratif di layar.
4.  Setelah selesai, MATLAB akan otomatis memanggil Simulink `CariLQR_PID.slx` untuk disimulasikan selama 30 detik.
5.  Hasil visualisasi berupa grafik **State vs Time**, **Output vs Time**, **Input Kontrol**, dan grafik **Konvergensi Biaya vs Iterasi** akan otomatis ditampilkan.

### 3. Langkah Optimasi Mandiri (Tanpa Simulink)
Jika Anda hanya ingin menjalankan perhitungan numerik dan visualisasi respons transien standar via ODE solver di MATLAB:
1.  Jalankan file `GA.m` di Command Window:
    ```matlab
    run('GA.m')
    ```
2.  Program akan mengoptimalkan gain LQR menggunakan GA dan memplot respon perbandingan posisi bodi mobil antara kontroler teroptimalkan dan kontroler baseline.

---

## 📈 Contoh Hasil Visualisasi

Setelah simulasi selesai berjalan, program akan menghasilkan grafik berikut untuk menganalisis performa suspensi:

1.  **Respon Output (Perpindahan Sprung Mass)**: Menunjukkan kemampuan sistem suspensi aktif teroptimasi dalam menekan perpindahan vertikal bodi mobil sehingga guncangan berkurang secara signifikan dibandingkan dengan kontroler baseline.
2.  **Upaya Kontrol (Control Effort)**: Menunjukkan profil gaya aktif yang dikerahkan oleh aktuator kontroler optimal vs baseline untuk memastikan aktuator bekerja dalam batas kapasitas fisiknya.
3.  **Kurva Konvergensi Biaya**: Menunjukkan penurunan nilai biaya fitness sepanjang generasi Algoritma Genetika hingga mencapai nilai minimum yang stabil.
