"""
Advanced Hybrid UPS / ESS Dashboard v3.1
"""
import sys, struct, time, threading
from datetime import datetime
from collections import deque
import numpy as np
import serial, serial.tools.list_ports
from PyQt6.QtWidgets import *
from PyQt6.QtCore import Qt, QTimer, QPropertyAnimation, QEasingCurve, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QColor
import pyqtgraph as pg

# ── 프로토콜 ──
CMD_FMT = '<BBffffHffBHBBffB'; CMD_SZ = struct.calcsize(CMD_FMT)
TELEM_FMT = '<BfffffffBBB'; TELEM_SZ = struct.calcsize(TELEM_FMT)
S_CMD=0xAA; S_TELEM=0xBB; END=0x55
CMD_SET=0x01; CMD_START=0x02; CMD_STOP=0x03; CMD_REQ=0x10
ST_RUN=0x01; ST_OV=0x02; ST_OC=0x04; ST_OT=0x08
ELEC_RATE = 120.0  # 원/kWh
BUF_SIZE = 72000

# ── Ring Buffer ──
class RBuf:
    def __init__(self, n):
        self.b=np.zeros(n); self.n=n; self.i=0; self.f=False
    def add(self,v):
        self.b[self.i]=v; self.i=(self.i+1)%self.n
        if self.i==0: self.f=True
    @property
    def cnt(self): return self.n if self.f else self.i
    def arr(self):
        if self.f: return np.concatenate((self.b[self.i:],self.b[:self.i]))
        return self.b[:self.i].copy()

# ── Serial Worker ──
class SW(QObject):
    telem=pyqtSignal(dict); conn=pyqtSignal(str); log=pyqtSignal(str)
    def __init__(self):
        super().__init__()
        self.sp=None; self.buf=bytearray(); self.rxb=0; self.txb=0; self.rxp=0; self.txp=0; self.err=0
        self.test_active = False
    def connect_port(self,port,baud):
        try:
            self.sp=serial.Serial(port,baud,timeout=0.1); self.buf.clear()
            self.conn.emit(f"연결됨: {port} @ {baud}"); self.log.emit(f"[{self._t()}] ✅ 연결 성공"); return True
        except Exception as e: self.conn.emit(f"실패: {e}"); return False
    def disconnect_port(self):
        if self.sp and self.sp.is_open: self.sp.close(); self.conn.emit("해제됨"); self.log.emit(f"[{self._t()}] ⏹ 해제")
    def send(self, cmd, *args):
        if not self.sp or not self.sp.is_open: return False
        pk = struct.pack(CMD_FMT, S_CMD, cmd, *args, END)
        try:
            self.sp.write(pk); self.txb+=len(pk); self.txp+=1
            n={CMD_SET:"SET",CMD_START:"START",CMD_STOP:"STOP",CMD_REQ:"REQ"}.get(cmd,"?")
            # 첫 3개 값으로 대표 로깅
            self.log.emit(f"[{self._t()}] ▶ TX [{n}] V={args[0]:.1f} I={args[1]:.1f} φ={args[2]:.1f}° (전체 {len(pk)}Bytes 전송됨)"); return True
        except: self.err+=1; return False
    def read(self):
        if not self.sp or not self.sp.is_open: return
        try:
            a=self.sp.in_waiting
            if a>0: c=self.sp.read(a); self.rxb+=len(c); self.buf.extend(c)
            while len(self.buf)>=TELEM_SZ:
                if self.buf[0]!=S_TELEM: self.buf.pop(0); continue
                if self.buf[TELEM_SZ-1]!=END: self.buf.pop(0); continue
                r=self.buf[:TELEM_SZ]; del self.buf[:TELEM_SZ]
                d=struct.unpack(TELEM_FMT,r); self.rxp+=1
                self.telem.emit({'v_hv':d[1],'i_hv':d[2],'v_lv':d[3],'i_lv':d[4],'temp':d[5],'duty':d[6],'phase':d[7],'st':d[8],'ts':time.time()})
        except: self.err+=1
    def pop(self):
        r=(self.rxb,self.txb,self.rxp,self.txp,self.err); self.rxb=self.txb=self.rxp=self.txp=self.err=0; return r
    @staticmethod
    def _t(): return datetime.now().strftime("%H:%M:%S.%f")[:-3]

# ── Energy Engine ──
class Eng:
    def __init__(self):
        self.grid_wh=0; self.ess_dis_wh=0; self.ess_chg_wh=0; self.load_wh=0; self.lt=None; self.state="STANDBY"
    def update(self,d,load_thr):
        t=d['ts']; p_hv=abs(d['v_hv']*d['i_hv']); p_lv=abs(d['v_lv']*d['i_lv'])
        if self.lt:
            dt=max(0,(t-self.lt))/3600.0
            self.load_wh+=p_lv*dt
            if d['st']&ST_RUN:
                if d['v_hv']<100:
                    self.state="UPS_BACKUP"; self.ess_dis_wh+=p_lv*dt
                elif p_lv>load_thr and p_hv>0:
                    self.state="PEAK_SHAVING"; self.grid_wh+=p_hv*dt; self.ess_dis_wh+=max(0,p_lv-p_hv)*dt
                else:
                    self.state="GRID_TIED"; self.grid_wh+=p_hv*dt
            else: self.state="STANDBY"
        self.lt=t
    @property
    def cover(self):
        t=self.grid_wh+self.ess_dis_wh; return (self.ess_dis_wh/t*100) if t>0.001 else 0
    @property
    def savings(self): return self.ess_dis_wh*ELEC_RATE/1000

# ── Stat Card ──
class Card(QFrame):
    def __init__(self,title,val="---",unit="",color="#38BDF8"):
        super().__init__()
        self.setObjectName("card")
        lo=QVBoxLayout(self); lo.setContentsMargins(14,10,14,10); lo.setSpacing(1)
        t=QLabel(title); t.setStyleSheet(f"color:#94A3B8;font-size:12px;font-weight:600;")
        self.v=QLabel(val); self.v.setStyleSheet(f"color:{color};font-size:26px;font-weight:800;font-family:Consolas,monospace;")
        u=QLabel(unit); u.setStyleSheet(f"color:#64748B;font-size:12px;")
        r=QHBoxLayout(); r.addWidget(self.v); r.addWidget(u,alignment=Qt.AlignmentFlag.AlignBottom); r.addStretch()
        lo.addWidget(t); lo.addLayout(r)
    def set(self,t): self.v.setText(t)

# ── Toast Notification ──
class Toast(QLabel):
    """화면 하단에 잠시 나타났다 사라지는 알림 배너."""
    def __init__(self, parent):
        super().__init__(parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setFixedHeight(40)
        self.setStyleSheet("background:#16A34A;color:white;font-weight:800;font-size:14px;border-radius:8px;padding:8px 20px;")
        self.hide()
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self.hide)

    def show_msg(self, text, color="#16A34A", ms=2500):
        self.setText(text)
        self.setStyleSheet(f"background:{color};color:white;font-weight:800;font-size:14px;border-radius:8px;padding:8px 20px;")
        self.show()
        self.raise_()
        # 부모 넓이에 맞추기
        pw = self.parent().width()
        self.setGeometry(20, self.parent().height()-60, pw-40, 40)
        self._timer.start(ms)

# ── Main GUI ──
class GUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("⚡ Advanced Hybrid UPS / ESS Dashboard")
        self.setMinimumSize(1400,900)
        ks='time v_hv i_hv v_lv i_lv temp duty p_load p_grid p_ess phase'.split()
        self.rb={k:RBuf(BUF_SIZE) for k in ks}; self.t0=None; self.sc=0
        self.sw=SW(); self.sw.telem.connect(self.on_telem); self.sw.conn.connect(self.on_conn); self.sw.log.connect(self.on_log)
        self.eng=Eng(); self.load_threshold=500.0
        self._style(); self._ui()
        self.rt=QTimer(); self.rt.timeout.connect(lambda:self.sw.read()); self.rt.start(50)
        self.st=QTimer(); self.st.timeout.connect(self._stats); self.st.start(1000)

    def _style(self):
        self.setStyleSheet("""
            *{font-family:'Segoe UI','Malgun Gothic',sans-serif;}
            QMainWindow,QWidget{background:#0F172A;color:#CBD5E1;}
            QFrame#card{background:#1E293B;border:1px solid #334155;border-radius:10px;}
            QGroupBox{background:#1E293B;border:1px solid #334155;border-radius:10px;margin-top:18px;padding:16px 10px 10px;font-weight:bold;color:#7DD3FC;font-size:14px;}
            QGroupBox::title{subcontrol-origin:margin;subcontrol-position:top left;left:14px;padding:0 6px;color:#7DD3FC;}
            QLabel{font-size:13px;color:#CBD5E1;}
            QPushButton{background:#334155;color:#F1F5F9;border:1px solid #475569;border-radius:8px;padding:10px 20px;font-weight:bold;font-size:14px;}
            QPushButton:hover{background:#475569;border-color:#94A3B8;}
            QPushButton:pressed{background:#1E293B;border:2px solid #94A3B8;}
            QPushButton#go{background:#16A34A;border:none;color:white;font-size:16px;font-weight:800;border-radius:10px;}
            QPushButton#go:hover{background:#22C55E;}
            QPushButton#go:pressed{background:#15803D;border:3px solid #86EFAC;}
            QPushButton#no{background:#DC2626;border:none;color:white;font-size:16px;font-weight:800;border-radius:10px;}
            QPushButton#no:hover{background:#EF4444;}
            QPushButton#no:pressed{background:#B91C1C;border:3px solid #FCA5A5;}
            QPushButton#ap{background:#2563EB;border:none;color:white;font-size:15px;font-weight:700;border-radius:10px;}
            QPushButton#ap:hover{background:#3B82F6;}
            QPushButton#ap:pressed{background:#1D4ED8;border:3px solid #93C5FD;}
            QComboBox{background:#1E293B;color:#E2E8F0;border:1px solid #334155;border-radius:6px;padding:6px 10px;font-size:13px;}
            QComboBox QAbstractItemView{background:#1E293B;color:#E2E8F0;selection-background-color:#334155;border:1px solid #475569;}
            QComboBox::drop-down{border:none;background:#334155;width:24px;border-top-right-radius:6px;border-bottom-right-radius:6px;}
            QSpinBox,QDoubleSpinBox{background:#1E293B;color:#E2E8F0;border:1px solid #334155;border-radius:6px;padding:6px 8px;font-size:13px;}
            QTabWidget::pane{border:1px solid #334155;background:#0F172A;border-radius:6px;}
            QTabBar::tab{background:#1E293B;color:#64748B;border:1px solid #334155;padding:10px 24px;font-weight:bold;font-size:14px;border-top-left-radius:6px;border-top-right-radius:6px;margin-right:2px;}
            QTabBar::tab:selected{background:#0F172A;color:#38BDF8;border-bottom:3px solid #38BDF8;}
            QTextEdit{background:#020617;color:#94A3B8;border:1px solid #1E293B;border-radius:6px;font-family:Consolas,monospace;font-size:12px;padding:6px;}
            QProgressBar{background:#1E293B;border:1px solid #334155;border-radius:8px;text-align:center;color:white;font-weight:800;font-size:14px;min-height:28px;}
            QProgressBar::chunk{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #16A34A,stop:1 #4ADE80);border-radius:8px;}
            QRadioButton{color:#CBD5E1;font-size:14px;font-weight:bold;}
            QRadioButton::indicator{width:16px;height:16px;}
            QScrollArea{border:none;}
        """)
        pg.setConfigOptions(antialias=True)
        pg.setConfigOption('background','#0F172A'); pg.setConfigOption('foreground','#94A3B8')

    def _ui(self):
        c=QWidget(); self.setCentralWidget(c); root=QVBoxLayout(c); root.setContentsMargins(8,4,8,4); root.setSpacing(4)
        # top bar
        top=QHBoxLayout()
        self.cp=QComboBox(); self._rp()
        br=QPushButton("🔄"); br.setFixedWidth(36); br.clicked.connect(self._rp)
        self.cb=QComboBox(); self.cb.addItems(["115200","230400","460800"])
        self.bc=QPushButton("🔌 연결"); self.bc.clicked.connect(self._tc)
        self.lc=QLabel("미연결"); self.lc.setStyleSheet("color:#F87171;font-weight:bold;padding-left:8px;font-size:13px;")
        self.ln=QLabel("Tx 0 B/s  Rx 0 B/s"); self.ln.setStyleSheet("color:#475569;font-family:Consolas;font-size:12px;")
        for w in [QLabel("포트:"),self.cp,br,QLabel("보율:"),self.cb,self.bc,self.lc]: top.addWidget(w)
        top.addStretch(); top.addWidget(self.ln)
        root.addLayout(top)
        # tabs
        tabs=QTabWidget()
        tabs.addTab(self._t1(),"📊 실시간 대시보드")
        tabs.addTab(self._t2(),"⚙️ 제어 / 설정")
        tabs.addTab(self._t3(),"📡 진단 / 로그")
        root.addWidget(tabs)

        # Toast notification overlay
        self.toast = Toast(c)

    # ═══ Tab 1: Dashboard ═══
    def _t1(self):
        w=QWidget(); lo=QVBoxLayout(w); lo.setSpacing(6)
        # row 1: UPS state + power cards
        r1=QHBoxLayout()
        self.lu=QLabel("⏸ 대기 (STANDBY)"); self.lu.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lu.setFixedHeight(72)
        self.lu.setStyleSheet("font-size:20px;font-weight:800;color:#94A3B8;background:#1E293B;border:1px solid #334155;border-radius:10px;padding:8px;")
        r1.addWidget(self.lu,stretch=2)
        self.cL=Card("부하 전력","0","W","#F97316"); self.cG=Card("그리드 전력","0","W","#38BDF8")
        self.cE=Card("ESS 전력","0","W","#4ADE80"); self.cC=Card("ESS 충당률","0","%","#A78BFA")
        for c in [self.cL,self.cG,self.cE,self.cC]: r1.addWidget(c,stretch=1)
        lo.addLayout(r1)
        # row 2: sensor cards
        r2=QHBoxLayout()
        self.cvh=Card("V_HV (그리드)","0","V","#FCA5A5"); self.cvl=Card("V_LV (배터리)","0","V","#86EFAC")
        self.cih=Card("I_HV","0","A","#D8B4FE"); self.cil=Card("I_LV","0","A","#93C5FD")
        self.ct=Card("변압기 온도","0","°C","#FDBA74"); self.cd=Card("PWM 듀티","0","%","#C4B5FD")
        self.cph=Card("위상 시프트","0","°","#E879F9")
        self.ckw=Card("누적 전력량","0","kWh","#FDE68A"); self.csv=Card("절감 금액","0","원","#34D399")
        for c in [self.cvh,self.cvl,self.cih,self.cil,self.ct,self.cd,self.cph,self.ckw,self.csv]: r2.addWidget(c,stretch=1)
        lo.addLayout(r2)
        # SOC
        rs=QHBoxLayout()
        rs.addWidget(QLabel("배터리 SOC"))
        self.bs=QProgressBar(); self.bs.setRange(0,100); self.bs.setValue(0); self.bs.setFormat("%p%")
        rs.addWidget(self.bs,stretch=1)
        lo.addLayout(rs)
        # graphs
        gl=QHBoxLayout()
        self.pp=pg.PlotWidget(title="전력 흐름 (W)"); self.pp.addLegend(offset=(10,10)); self.pp.showGrid(x=True,y=True,alpha=0.12)
        self.pp.setLabel('left','Power','W'); self.pp.setLabel('bottom','Time','s')
        self.cpL=self.pp.plot(pen=pg.mkPen('#F97316',width=2),name='부하')
        self.cpG=self.pp.plot(pen=pg.mkPen('#38BDF8',width=2),name='그리드')
        self.cpE=self.pp.plot(pen=pg.mkPen('#4ADE80',width=2),name='ESS')
        gl.addWidget(self.pp)
        self.pt=pg.PlotWidget(title="온도 (°C) / 듀티 (%)"); self.pt.addLegend(offset=(10,10)); self.pt.showGrid(x=True,y=True,alpha=0.12)
        self.pt.setLabel('left','Value'); self.pt.setLabel('bottom','Time','s')
        self.cT=self.pt.plot(pen=pg.mkPen('#FDBA74',width=2),name='Temp (°C)')
        self.cD=self.pt.plot(pen=pg.mkPen('#C4B5FD',width=2,style=Qt.PenStyle.DashLine),name='Duty (%)')
        gl.addWidget(self.pt)
        lo.addLayout(gl,stretch=1)
        return w

    # ═══ Tab 2: Settings ═══
    def _t2(self):
        sc=QScrollArea(); sc.setWidgetResizable(True)
        inner=QWidget(); inner.setStyleSheet("background:#0F172A;")
        lo=QVBoxLayout(inner); lo.setSpacing(14)

        # 운전 제어 (맨 위로)
        gR=QGroupBox("운전 제어"); lR=QHBoxLayout(gR)
        b1=QPushButton("▶  운전 시작"); b1.setObjectName("go"); b1.setMinimumHeight(50); b1.setCursor(Qt.CursorShape.PointingHandCursor); b1.clicked.connect(self._go)
        b2=QPushButton("⏹  운전 정지"); b2.setObjectName("no"); b2.setMinimumHeight(50); b2.setCursor(Qt.CursorShape.PointingHandCursor); b2.clicked.connect(self._no)
        b3=QPushButton("💾  파라미터 전송"); b3.setObjectName("ap"); b3.setMinimumHeight(50); b3.setCursor(Qt.CursorShape.PointingHandCursor); b3.clicked.connect(self._ap)
        lR.addWidget(b1); lR.addWidget(b2); lR.addWidget(b3)
        lo.addWidget(gR)

        # 알림 배너
        self.lbl_notify = QLabel("")
        self.lbl_notify.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_notify.setFixedHeight(0)
        self.lbl_notify.setStyleSheet("background:transparent;font-size:0px;")
        lo.addWidget(self.lbl_notify)

        # 운전 모드
        gM=QGroupBox("DAB 운전 모드"); lM=QHBoxLayout(gM)
        self.bm=QButtonGroup()
        for k,n in {1:"SPS",2:"EPS",3:"DPS",4:"TPS"}.items():
            rb=QRadioButton(n)
            if k==1: rb.setChecked(True)
            self.bm.addButton(rb,k); lM.addWidget(rb)
        lM.addStretch()
        lo.addWidget(gM)

        # 전력 목표
        gP=QGroupBox("전력 제어 목표치"); lP=QGridLayout(gP)
        self.sv=QDoubleSpinBox(); self.sv.setRange(0,500); self.sv.setValue(48.0); self.sv.setDecimals(1)
        self.si=QDoubleSpinBox(); self.si.setRange(0,100); self.si.setValue(20.0); self.si.setDecimals(1)
        self.sph=QDoubleSpinBox(); self.sph.setRange(-180,180); self.sph.setValue(30.0); self.sph.setDecimals(1)
        lP.addWidget(QLabel("목표 전압 (V):"),0,0); lP.addWidget(self.sv,0,1)
        lP.addWidget(QLabel("전류 제한 (A):"),0,2); lP.addWidget(self.si,0,3)
        lP.addWidget(QLabel("위상 시프트 (°):"),1,0); lP.addWidget(self.sph,1,1)
        lo.addWidget(gP)

        # ESS / 부하 임계값
        gE=QGroupBox("ESS 동작 설정 (부하 임계값 / 운전 시간)"); lE=QGridLayout(gE)
        self.spin_load_thr=QDoubleSpinBox(); self.spin_load_thr.setRange(0,10000); self.spin_load_thr.setValue(500)
        self.spin_load_thr.setSuffix(" W"); self.spin_load_thr.setDecimals(0)
        self.spin_load_thr.valueChanged.connect(lambda v: setattr(self,'load_threshold',v))
        self.spin_ess_time=QSpinBox(); self.spin_ess_time.setRange(0,3600); self.spin_ess_time.setValue(60)
        self.spin_ess_time.setSuffix(" 초")
        lE.addWidget(QLabel("부하 임계값 (이상 시 ESS 투입):"),0,0); lE.addWidget(self.spin_load_thr,0,1)
        lE.addWidget(QLabel("ESS 최대 운전 시간:"),0,2); lE.addWidget(self.spin_ess_time,0,3)
        lo.addWidget(gE)

        # 배터리 프로파일
        gB=QGroupBox("배터리 프로파일"); lB=QGridLayout(gB)
        self.cbt=QComboBox()
        for k,v in {1:"Li-ion",2:"LFP",3:"LTO",4:"Lead-Acid"}.items(): self.cbt.addItem(f"{k}: {v}",k)
        self.suvp=QDoubleSpinBox(); self.suvp.setRange(0,400); self.suvp.setValue(36.0)
        self.sovp=QDoubleSpinBox(); self.sovp.setRange(0,400); self.sovp.setValue(58.4)
        self.ssoc=QSpinBox(); self.ssoc.setRange(0,100); self.ssoc.setValue(90)
        self.sdt=QSpinBox(); self.sdt.setRange(0,500); self.sdt.setValue(200)
        lB.addWidget(QLabel("배터리 타입:"),0,0); lB.addWidget(self.cbt,0,1)
        lB.addWidget(QLabel("목표 SOC (%):"),0,2); lB.addWidget(self.ssoc,0,3)
        lB.addWidget(QLabel("UVP (V):"),1,0); lB.addWidget(self.suvp,1,1)
        lB.addWidget(QLabel("OVP (V):"),1,2); lB.addWidget(self.sovp,1,3)
        lB.addWidget(QLabel("Dead Time (ns):"),2,0); lB.addWidget(self.sdt,2,1)
        lo.addWidget(gB)

        # 수동 시뮬레이션 설정 (S/W 더미 데이터)
        gS=QGroupBox("수동 장비 시뮬레이션 (수식 배제, 강제 설정)"); lS=QGridLayout(gS)
        self.spin_sim_load=QDoubleSpinBox(); self.spin_sim_load.setRange(0, 50000); self.spin_sim_load.setValue(300.0)
        self.spin_sim_load.setSuffix(" W"); self.spin_sim_load.setDecimals(1)
        self.spin_sim_ess=QDoubleSpinBox(); self.spin_sim_ess.setRange(0, 50000); self.spin_sim_ess.setValue(0.0)
        self.spin_sim_ess.setSuffix(" W"); self.spin_sim_ess.setDecimals(1)
        lS.addWidget(QLabel("시뮬레이션 부하 전력:"),0,0); lS.addWidget(self.spin_sim_load,0,1)
        lS.addWidget(QLabel("시뮬레이션 ESS 출력:"),0,2); lS.addWidget(self.spin_sim_ess,0,3)
        lo.addWidget(gS)

        lo.addStretch()
        sc.setWidget(inner); return sc

    # ═══ Tab 3: Diagnostics ═══
    def _t3(self):
        w=QWidget(); lo=QVBoxLayout(w)
        top=QHBoxLayout()
        bc=QPushButton("로그 지우기"); bc.setFixedWidth(120); bc.clicked.connect(lambda: self.lt.clear())
        bt=QPushButton("🚀 연결 속도 테스트 (5초)"); bt.setStyleSheet("background:#9333EA;border:none;color:white;font-weight:bold;border-radius:6px;")
        bt.clicked.connect(self._test_speed)
        top.addWidget(bt); top.addStretch(); top.addWidget(bc)
        lo.addLayout(top)
        self.lt=QTextEdit(); self.lt.setReadOnly(True); lo.addWidget(self.lt)
        return w

    # ── Actions ──
    def _rp(self):
        self.cp.clear()
        for p in serial.tools.list_ports.comports(): self.cp.addItem(f"{p.device} - {p.description}",p.device)

    def _tc(self):
        if self.sw.sp and self.sw.sp.is_open: self.sw.disconnect_port(); self.bc.setText("🔌 연결")
        else:
            port=self.cp.currentData()
            if not port: return
            if self.sw.connect_port(port,int(self.cb.currentText())): self.bc.setText("🚫 해제")

    def _gp(self): 
        # C struct 매칭: target_v, current_limit, phase_shift, load_th, ess_time, uvp, ovp, soc, dt, battery_type, sim_load, sim_ess
        return (
            self.sv.value(), self.si.value(), self.sph.value(),
            self.spin_load_thr.value(), self.spin_ess_time.value(),
            self.suvp.value(), self.sovp.value(), self.ssoc.value(), self.sdt.value(),
            self.cbt.currentData(),
            self.spin_sim_load.value(), self.spin_sim_ess.value()
        )

    def _ap(self):
        args = self._gp()
        ok = self.sw.send(CMD_SET, *args)
        self.load_threshold = args[3]
        if ok:
            self.toast.show_msg(f"✅ 파라미터 전부 STM32 전송 완료 (V={args[0]:.1f} I={args[1]:.1f} φ={args[2]:.1f}°)","#16A34A")
        else:
            self.toast.show_msg("❌ 전송 실패 — 포트 미연결 (PC 로컬만 적용)","#F59E0B")

    def _go(self):
        args = self._gp()
        ok = self.sw.send(CMD_START, *args)
        self.load_threshold = args[3]
        if ok:
            self.toast.show_msg("▶ 운전 시작 명령 및 파라미터 전송됨","#16A34A")
        else:
            self.toast.show_msg("❌ 전송 실패 — 포트가 연결되지 않았습니다","#DC2626")

    def _no(self):
        args = self._gp()
        ok = self.sw.send(CMD_STOP, *args)
        if ok:
            self.toast.show_msg("⏹ 운전 정지 명령 전송됨","#F59E0B")
        else:
            self.toast.show_msg("❌ 전송 실패 — 포트가 연결되지 않았습니다","#DC2626")

    def _test_speed(self):
        if not self.sw.sp or not self.sw.sp.is_open:
            self.toast.show_msg("❌ 미연결 상태입니다. 포트를 연결해주세요.","#DC2626")
            return
        if self.sw.test_active: return
        self.sw.test_active = True
        self.toast.show_msg("🚀 연결 속도 측정 중... (5초)","#9333EA", 5000)
        self.on_log(f"[{SW._t()}] 🚀 최대 송수신 속도 측정 시작...")
        
        args = self._gp()
        def ping():
            t0 = time.time()
            cnt = 0
            while time.time() - t0 < 5.0 and self.sw.sp.is_open:
                # REQ_TELEM을 빠르게 매우 많이 전송 (대역폭 스트레스 테스트)
                self.sw.send(CMD_REQ, *args)
                cnt += 1
                time.sleep(0.002) # 짧은 인터벌 부여
            self.sw.test_active = False
            self.toast.show_msg("✅ 연결 테스트 종료. 통신 상태를 확인하세요.","#16A34A")
            self.on_log(f"[{SW._t()}] 🚀 테스트 종료. 5초간 총 {cnt}회 (약 {cnt*CMD_SZ} Bytes) 송신 시도 완료.")
        
        threading.Thread(target=ping, daemon=True).start()

    def _stats(self):
        rx_b,tx_b,rx_p,tx_p,err=self.sw.pop()
        self.ln.setText(f"Tx {tx_b} B/s ({tx_p}pkt) | Rx {rx_b} B/s ({rx_p}pkt) | Err {err}")

    # ── Telemetry handler ──
    def on_telem(self,d):
        if not self.t0: self.t0=d['ts']
        t=d['ts']-self.t0; self.sc+=1
        p_hv=abs(d['v_hv']*d['i_hv']); p_lv=abs(d['v_lv']*d['i_lv'])
        # ESS 전력: 부하가 임계값 초과 시에만 ESS가 커버
        load_w=p_lv; grid_w=p_hv
        ess_w=max(0,load_w-grid_w) if (d['st']&ST_RUN and load_w>self.load_threshold) else 0.0

        for k,v in [('time',t),('v_hv',d['v_hv']),('i_hv',d['i_hv']),('v_lv',d['v_lv']),('i_lv',d['i_lv']),
                    ('temp',d['temp']),('duty',d['duty']*100),('p_load',load_w),('p_grid',grid_w),('p_ess',ess_w),('phase',d['phase'])]:
            self.rb[k].add(v)

        self.eng.update(d,self.load_threshold)
        # cards
        self.cvh.set(f"{d['v_hv']:.1f}"); self.cvl.set(f"{d['v_lv']:.1f}")
        self.cih.set(f"{d['i_hv']:.2f}"); self.cil.set(f"{d['i_lv']:.2f}")
        self.ct.set(f"{d['temp']:.1f}"); self.cd.set(f"{d['duty']*100:.1f}")
        self.cph.set(f"{d['phase']:.1f}")
        self.cL.set(f"{load_w:.0f}"); self.cG.set(f"{grid_w:.0f}")
        self.cE.set(f"{ess_w:.0f}"); self.cC.set(f"{self.eng.cover:.1f}")
        self.ckw.set(f"{self.eng.load_wh/1000:.3f}"); self.csv.set(f"{self.eng.savings:.0f}")
        # SOC
        soc=max(0,min(100,(d['v_lv']-36)/(48-36)*100))
        self.bs.setValue(int(soc))
        # UPS state
        sm={"STANDBY":("⏸ 대기 (STANDBY)","#94A3B8","#1E293B"),"INIT":("🔄 초기화","#94A3B8","#1E293B"),
            "GRID_TIED":("🟢 그리드 연결 (충전/대기)","#4ADE80","#052E16"),
            "PEAK_SHAVING":("🟡 피크 저감 (ESS+Grid 분담)","#FBBF24","#422006"),
            "UPS_BACKUP":("🔴 정전 백업 (ESS 단독)","#F87171","#450A0A")}
        txt,fg,bg=sm.get(self.eng.state,sm["STANDBY"])
        self.lu.setText(txt); self.lu.setStyleSheet(f"font-size:20px;font-weight:800;color:{fg};background:{bg};border:1px solid #334155;border-radius:10px;padding:8px;")
        # graphs (every 10 samples for perf)
        if self.sc%10==0:
            ta=self.rb['time'].arr()
            self.cpL.setData(ta,self.rb['p_load'].arr()); self.cpG.setData(ta,self.rb['p_grid'].arr()); self.cpE.setData(ta,self.rb['p_ess'].arr())
            self.cT.setData(ta,self.rb['temp'].arr()); self.cD.setData(ta,self.rb['duty'].arr())

    def on_conn(self,m):
        self.lc.setText(m); c="#4ADE80" if "연결됨" in m else "#F87171"
        self.lc.setStyleSheet(f"color:{c};font-weight:bold;padding-left:8px;font-size:13px;")

    def on_log(self,m):
        self.lt.append(m); self.lt.verticalScrollBar().setValue(self.lt.verticalScrollBar().maximum())

    def closeEvent(self,e): self.sw.disconnect_port(); self.rt.stop(); self.st.stop(); e.accept()

if __name__=="__main__":
    app=QApplication(sys.argv); w=GUI(); w.show(); sys.exit(app.exec())
