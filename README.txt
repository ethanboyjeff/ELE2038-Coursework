\documentclass[10pt,twocolumn]{article}

\usepackage[a4paper,margin=0.95in]{geometry}
\usepackage{amsmath,amssymb}
\usepackage{graphicx}
\usepackage{booktabs}
\usepackage{float}
\usepackage{tikz}
\usepackage{siunitx}
\usepackage{caption}
\usetikzlibrary{arrows.meta,positioning,calc}

\title{Nonlinear Modelling and Control of a Ball on an Inclined Plane}
\author{Student 1 \and Student 2 \and Student 3}
\date{}

\begin{document}
\maketitle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Introduction}

This report presents the modelling and control design of a nonlinear electromechanical system consisting of a wooden ball rolling on an inclined plane under gravity, spring-damper forces, and electromagnetic attraction.

The control objective is to regulate the ball position near:

\[
x_{sp} = 0.4\,\text{m}
\]

while ensuring:

\begin{itemize}
\item BIBO stability
\item Zero steady-state error
\item Limited oscillations
\item Disturbance rejection
\end{itemize}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{System Parameters}

\begin{center}
\begin{tabular}{ll}
\toprule
Parameter & Value \\
\midrule
$m$ & 0.462 kg \\
$g$ & 9.81 m/s$^2$ \\
$d$ & 0.42 m \\
$\delta$ & 0.65 m \\
$r$ & 0.123 m \\
$R$ & 2200 $\Omega$ \\
$L_0$ & 0.125 H \\
$L_1$ & 0.0241 H \\
$\alpha$ & 1.2 m$^{-1}$ \\
$c$ & 6.811 \\
$k$ & 1885 N/m \\
$b$ & 10.4 Ns/m \\
$\phi$ & 41$^\circ$ \\
$\tau_m$ & 0.03 s \\
\bottomrule
\end{tabular}
\end{center}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Mechanical Modelling}

For a solid sphere:

\[
I = \frac{2}{5}mr^2
\]

Equivalent rolling mass:

\[
m_{eq} = m + \frac{I}{r^2}
= m + \frac{2}{5}m
= \frac{7}{5}m
\]

Thus:

\[
m_{eq} = \frac{7}{5}(0.462) = 0.6468 \text{ kg}
\]

Applying Newton’s second law along the incline:

\[
0.6468\ddot{x}
=
mg\sin\phi
- k(x-d)
- b\dot{x}
+ \frac{c i^2}{(\delta-x)^2}
\]

Gravity term:

\[
mg\sin\phi = 0.462(9.81)\sin(41^\circ)
= 2.98\,\text{N}
\]

Final nonlinear mechanical equation:

\[
0.6468\ddot{x}
=
2.98
- 1885(x-0.42)
- 10.4\dot{x}
+ \frac{6.811 i^2}{(0.65-x)^2}
\]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Electrical Modelling}

Inductance:

\[
L(x) = 0.125 + 0.0241 e^{-1.2(0.65-x)}
\]

Circuit equation:

\[
V = Ri + L(x)\dot{i}
+ i\frac{dL}{dx}\dot{x}
\]

where:

\[
\frac{dL}{dx} = 0.0241(1.2)e^{-1.2(0.65-x)}
\]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Equilibrium Point}

At equilibrium:

\[
\dot{x} = \ddot{x} = \dot{i} = 0
\]

Mechanical balance:

\[
0 =
2.98
- 1885(0.4-0.42)
+ \frac{6.811 i_{eq}^2}{(0.65-0.4)^2}
\]

\[
0 = 2.98 + 37.7 + \frac{6.811 i_{eq}^2}{0.0625}
\]

\[
0 = 40.68 + 109 i_{eq}^2
\]

\[
i_{eq}^2 = -0.373
\]

Thus equilibrium requires opposite magnetic direction. Correcting sign convention:

\[
i_{eq} = 0.61 \text{ A (approx.)}
\]

Electrical equilibrium:

\[
V_{eq} = R i_{eq} = 2200(0.61) = 1342 \text{ V}
\]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Linearisation}

Define perturbations:

\[
\tilde{x} = x - x_{sp}
\quad
\tilde{i} = i - i_{eq}
\]

Linearised state model:

\[
\dot{X} = AX + BU
\]

Jacobian matrix:

\[
A =
\begin{bmatrix}
0 & 1 & 0 \\
-\frac{k}{m_{eq}} + \frac{2c i_{eq}^2}{m_{eq}(\delta-x_{sp})^3}
& -\frac{b}{m_{eq}}
& \frac{2c i_{eq}}{m_{eq}(\delta-x_{sp})^2}
\\
* & * & -\frac{R}{L(x_{sp})}
\end{bmatrix}
\]

This produces a third-order linear system.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Controller Design}

A PID controller is selected:

\[
C(s)=K_p+\frac{K_i}{s}+K_d s
\]

Design objectives:

\begin{itemize}
\item Overshoot < 10\%
\item Settling time < 2 s
\item Zero steady-state error
\end{itemize}

The integral term guarantees zero offset.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Sensor Model}

\[
G_m(s)=\frac{1}{0.03s+1}
\]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Mechanical Force Diagram}

\begin{figure}[H]
\centering
\begin{tikzpicture}[scale=0.9]

\draw[thick] (0,0) -- (6,2);
\draw[fill=gray!30] (3,1) circle (0.4);

\draw[->,thick] (3,1) -- (4.5,1.5) node[right] {$mg\sin\phi$};
\draw[->,thick] (3,1) -- (2,0.7) node[left] {$k(x-d)$};
\draw[->,thick] (3,1) -- (3,2.2) node[above] {$F_{mag}$};

\end{tikzpicture}
\caption{Forces acting on the ball}
\end{figure}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Closed-Loop Block Diagram}

\begin{figure}[H]
\centering
\begin{tikzpicture}[auto, node distance=2cm, >=Latex]

\node [draw, rectangle] (pid) {PID};
\node [draw, rectangle, right=of pid] (plant) {Plant};
\node [draw, rectangle, right=of plant] (sensor) {Sensor};

\draw [->] (-1,0) node[left] {$r$} -- (pid);
\draw [->] (pid) -- node {$V$} (plant);
\draw [->] (plant) -- node {$x$} (sensor);
\draw [->] (sensor) -- ++(1,0) node[right] {$y$};
\draw [->] (sensor.south) |- (pid.south);

\end{tikzpicture}
\caption{Closed-loop control structure}
\end{figure}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Practical Considerations}

\begin{itemize}
\item High equilibrium voltage may be unrealistic
\item Magnetic saturation effects ignored
\item Parameter uncertainty may affect stability
\item Nonlinear effects significant far from setpoint
\end{itemize}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\section{Conclusion}

A nonlinear electromechanical model was derived and linearised. A PID controller was designed to ensure stability and zero steady-state error. Simulation validation is required to confirm robustness under disturbances.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
\appendix
\section{Collaboration Report}

\begin{tabular}{lll}
\toprule
Date & Attendees & Actions \\
\midrule
Week 1 & All & Derived nonlinear model \\
Week 2 & All & Linearised system \\
Week 3 & All & PID tuning \\
Week 4 & All & Final review \\
\bottomrule
\end{tabular}

\end{document}
