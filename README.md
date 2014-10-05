autotune
========

Arduino software to control an autotener

The autotuner is the kitset from the EB104.com website.
Any tuner of the LC type with 8 shunt capacitors and 8
series inductors plus facility to switch capacitors
to either end of inductors will do the job. The relays
should be 12 volt or 24 volt types.

The output of a Stockton or Breune style bridge are
connected to the forward and reverse power inputs
(A6, A7) of the Arduino for VSWR detection.

The 8 capacitor relays are driven from D2 ... D9 via
a ULN2003 driver chip

The 8 inductor relays are driven from D12 ... D13 plus
A0 ... A5 which are set up as digital outputs. Again
the relays are interfaced via a ULN2003 driver chip.

D10 is connected to a pushbutton to initiate a tuning
sequence. RF should be applied to the tuner then the
button pressed. A course tune sequence is initiated
where the capacitors are stepped first looking for
best SWR and holding on best capacitor. The inductors
are similarly stepped and held on the inductor prodicing
the lowest SWR.

A fine tune sequence is established to set exact capacitor
value followed by best inductor. A reiteration is done
to get the final values and the tuner waits for another
button press.
