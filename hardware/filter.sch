<Qucs Schematic 0.0.19>
<Properties>
  <View=0,0,1200,800,1,0,0>
  <Grid=10,10,1>
  <DataSet=filter.dat>
  <DataDisplay=filter.dpl>
  <OpenDisplay=1>
  <Script=filter.m>
  <RunScript=0>
  <showFrame=0>
  <FrameText0=Title>
  <FrameText1=Drawn By:>
  <FrameText2=Date:>
  <FrameText3=Revision:>
</Properties>
<Symbol>
</Symbol>
<Components>
<Pac P1 1 100 470 18 -26 0 1 "1" 1 "50 Ohm" 1 "0 dBm" 0 "1 GHz" 0 "26.85" 0>
<GND * 1 100 540 0 0 0 0>
<C C1 1 170 400 -30 -50 0 0 "59.00p" 1 "" 0 "neutral" 0>
<C C2 1 240 470 17 -26 0 1 "698.0p" 1 "" 0 "neutral" 0>
<GND * 1 240 540 0 0 0 0>
<L L2 1 380 470 17 -26 0 1 "100.0n" 1 "" 0 "neutral" 0>
<GND * 1 380 540 0 0 0 0>
<C C3 1 450 400 -30 -50 0 0 "13.70p" 1 "" 0 "neutral" 0>
<C C4 1 520 470 17 -26 0 1 "750.0p" 1 "" 0 "neutral" 0>
<GND * 1 520 540 0 0 0 0>
<L L4 1 660 470 17 -26 0 1 "100.0n" 1 "" 0 "neutral" 0>
<GND * 1 660 540 0 0 0 0>
<C C5 1 730 400 -30 -50 0 0 "11.00p" 1 "" 0 "neutral" 0>
<C C6 1 800 470 17 -26 0 1 "750.0p" 1 "" 0 "neutral" 0>
<GND * 1 800 540 0 0 0 0>
<L L6 1 940 470 17 -26 0 1 "100.0n" 1 "" 0 "neutral" 0>
<GND * 1 940 540 0 0 0 0>
<C C7 1 1010 400 -30 -50 0 0 "11.50p" 1 "" 0 "neutral" 0>
<C C8 1 1080 470 17 -26 0 1 "732.0p" 1 "" 0 "neutral" 0>
<GND * 1 1080 540 0 0 0 0>
<L L8 1 1220 470 17 -26 0 1 "100.0n" 1 "" 0 "neutral" 0>
<GND * 1 1220 540 0 0 0 0>
<C C9 1 1290 400 -30 -50 0 0 "32.40p" 1 "" 0 "neutral" 0>
<Pac P2 1 1360 470 18 -26 0 1 "2" 1 "72 Ohm" 1 "0 dBm" 0 "1 GHz" 0 "26.85" 0>
<GND * 1 1360 540 0 0 0 0>
<.SP SP1 1 650 100 0 67 0 0 "log" 1 "13.70MHz" 1 "23.91MHz" 1 "1001" 1 "no" 0 "1" 0 "2" 0 "no" 0 "no" 0>
<Eqn Eqn1 1 900 170 -28 15 0 0 "dBS21=dB(S[2,1])" 1 "dBS11=dB(S[1,1])" 1 "group_delay=-diff(unwrap(angle(S[2,1])),2*pi*frequency)" 1 "yes" 0>
</Components>
<Wires>
<100 400 100 440 "" 0 0 0 "">
<100 500 100 540 "" 0 0 0 "">
<100 400 140 400 "" 0 0 0 "">
<200 400 240 400 "" 0 0 0 "">
<240 400 240 440 "" 0 0 0 "">
<240 500 240 540 "" 0 0 0 "">
<240 400 380 400 "" 0 0 0 "">
<380 400 380 440 "" 0 0 0 "">
<380 500 380 540 "" 0 0 0 "">
<380 400 420 400 "" 0 0 0 "">
<480 400 520 400 "" 0 0 0 "">
<520 400 520 440 "" 0 0 0 "">
<520 500 520 540 "" 0 0 0 "">
<520 400 660 400 "" 0 0 0 "">
<660 400 660 440 "" 0 0 0 "">
<660 500 660 540 "" 0 0 0 "">
<660 400 700 400 "" 0 0 0 "">
<760 400 800 400 "" 0 0 0 "">
<800 400 800 440 "" 0 0 0 "">
<800 500 800 540 "" 0 0 0 "">
<800 400 940 400 "" 0 0 0 "">
<940 400 940 440 "" 0 0 0 "">
<940 500 940 540 "" 0 0 0 "">
<940 400 980 400 "" 0 0 0 "">
<1040 400 1080 400 "" 0 0 0 "">
<1080 400 1080 440 "" 0 0 0 "">
<1080 500 1080 540 "" 0 0 0 "">
<1080 400 1220 400 "" 0 0 0 "">
<1220 400 1220 440 "" 0 0 0 "">
<1220 500 1220 540 "" 0 0 0 "">
<1220 400 1260 400 "" 0 0 0 "">
<1320 400 1360 400 "" 0 0 0 "">
<1360 400 1360 440 "" 0 0 0 "">
<1360 500 1360 540 "" 0 0 0 "">
</Wires>
<Diagrams>
</Diagrams>
<Paintings>
<Text 100 100 12 #000000 0 "4th Order Chebyshev Bandpass\nDirect-Coupled, Series Capacitor\nLower Cutoff Freq. = 17.9 MHz; Upper Cutoff Freq. = 18.3 MHz\nPassband Ripple = 0.5 dB\n\nrf-tools.com | Nov 29 2021 21:51">
</Paintings>