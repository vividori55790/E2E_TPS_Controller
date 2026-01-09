# **⚡ DAB Converter Simulink Project**

이 프로젝트는 Dual Active Bridge (DAB) DC-DC 컨버터의 시뮬레이션 모델입니다.  
PSIM 설계를 기반으로 Simulink로 이식되었으며, 다중 사용자 협업을 위해 모듈화(Modularization) 및 프로젝트(Project) 기반으로 구성되어 있습니다.

## **🚀 Quick Start (시작하는 법)**

1. **DAB\_Converter\_Prj.prj** 파일을 더블 클릭하여 MATLAB을 실행합니다.  
   * *자동으로 경로(Path)가 설정되고 필요한 데이터가 로드됩니다.*  
2. Scripts 폴더의 **DAB\_Control\_Center.mlx** 파일을 엽니다.  
3. 스크립트 내의 **"시뮬레이션 실행"** 섹션을 실행(Run)하여 결과를 확인합니다.

## **📂 폴더 및 파일 구조 (Directory Structure)**

이 프로젝트는 **기능 단위로 파일을 분리**하여 Git 충돌을 최소화하도록 설계되었습니다.

### **1\. Root (최상위 경로)**

* **DAB\_Converter\_Prj.prj** (Simulink Project)  
  * ⭐️ **프로젝트 진입점**입니다. 반드시 이 파일을 통해 프로젝트를 열어야 합니다.  
  * 경로 자동 설정, Git 연동, 시작 스크립트 실행을 담당합니다.  
* **Main\_Simulation.slx** (Integration Model)  
  * **역할:** \[마더보드\]  
  * 실제 로직은 없고, Circuit\_Model과 Control\_Logic을 연결하는 껍데기 파일입니다.  
  * **수정 규칙:** 블록 간의 배선(Wiring)을 바꿀 때만 수정합니다.

### **2\. Modules/ (핵심 모델)**

* **Circuit\_Model.slx** (Plant Model)  
  * **역할:** \[회로 하드웨어\]  
  * MOSFET H-Bridge, 고주파 변압기, $L, C$ 필터, 배터리 모델이 포함됩니다.  
  * **Input:** Gate 신호 / **Output:** 센서 측정값 ($V, I$)  
  * *담당: 회로 설계 엔지니어*  
* **Control\_Logic.slx** (Controller Model)  
  * **역할:** \[제어기 두뇌\]  
  * PI 제어기, 위상 천이(Phase Shift) 계산, PWM 생성 로직이 포함됩니다.  
  * **Input:** 센서 측정값 / **Output:** Gate 신호  
  * *담당: 제어 알고리즘 엔지니어*

### **3\. Data/ (파라미터 관리)**

* **DAB\_Parameters.sldd** (Simulink Data Dictionary)  
  * **역할:** \[공통 데이터베이스\]  
  * 하드웨어 불변 상수($R, L, C, V\_{nom}$)를 저장합니다.  
  * **규칙:** 팀원 전체가 합의된 하드웨어 스펙만 저장합니다. (함부로 수정 금지 🚫)

### **4\. Scripts/ (제어 및 분석)**

* **DAB\_Control\_Center.mlx** (Live Script)  
  * **역할:** \[통합 조종석\]  
  * 시뮬레이션 조건 설정, 실행(sim), 결과 그래프 출력(plot)을 담당합니다.  
  * **규칙:** 시뮬레이션 테스트를 할 때 주로 이 파일을 사용합니다.

## **🤝 협업 가이드 (Collaboration Workflow)**

충돌을 방지하기 위해 아래의 워크플로우를 준수해 주세요.

### **역할 분담에 따른 작업 위치**

| 역할 | 수정해야 할 파일 | 주요 작업 내용 |
| :---- | :---- | :---- |
| **회로 담당** | Modules/Circuit\_Model.slx | MOSFET 파라미터 변경, 회로 결선 수정 |
| **제어 담당** | Modules/Control\_Logic.slx | PI 게인 튜닝, 제어 알고리즘 변경 |
| **스펙 관리** | Data/DAB\_Parameters.sldd | 인덕턴스($L$), 커패시턴스($C$) 값 변경 |
| **테스트/분석** | Scripts/DAB\_Control\_Center.mlx | 시나리오별 시뮬레이션 돌리기, 파형 분석 |

### **⚠️ 주의사항 (SLDD vs MLX)**

* **SLDD (.sldd):** "우리 제품의 하드웨어 스펙은 이거야\!" (공유용/고정값)  
* **MLX (.mlx):** "이번엔 부하를 바꿔서 테스트 해볼까?" (실험용/가변값)  
* **충돌 방지:** .mlx 파일 내부에서 .sldd에 이미 정의된 변수(예: L\_leakage)를 L\_leakage \= ... 형태로 재정의하지 마세요. (상수를 덮어쓰면 혼란 발생)

## **🛠 Prerequisites (필수 환경)**

* MATLAB & Simulink R2023b 이상 권장  
* **Simscape Electrical** 툴박스 필수  
* **Simulink Project** 기능 활성화

### **📬 Contact**

프로젝트 구조나 Git 사용법에 대해 궁금한 점이 있다면 \*\*\[관리자 이름/이메일\]\*\*에게 문의하세요.
