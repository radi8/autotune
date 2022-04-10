
# Autotune [![Badge License]][License]

*Control An Autotuner With Arduino Code*

<br>

The **Autotuner** is the kitset from the `EB104.com` website.

<br>

## Requirements

Any tuner of the LC type with 8 shunt capacitors and <br>
8 series inductors plus facility to switch capacitors to <br>
either end of inductors will do the job.

The relays should be `12V` / `24V` types.

<br>

## Connections

The output of a Stockton or Breune style bridge are
connected to the forward and reverse power inputs
(A6, A7) of the Arduino for VSWR detection.

The 8 capacitor relays are driven from D2 ... D9 via
a ULN2003 driver chip

The 8 inductor relays are driven from D12 ... D19 pins.
(D14 ... D19 = A0 ... A5) which are set up as digital
outputs. Again the relays are interfaced via a ULN2003
driver chip.

D10 is connected to a pushbutton to initiate a tuning
sequence. RF should be applied to the tuner then the
button pressed. A course tune sequence is initiated
where the capacitors are stepped first looking for
best SWR and holding on best capacitor. The inductors
are similarly stepped and held on the inductor prodicing
the lowest SWR.

D11 is an output to change the capacitor relay set from
end to end of the series inductors.

A fine tune sequence is established to set exact capacitor
value followed by best inductor. A reiteration is done
to get the final values and the tuner waits for another
button press.

Bit out of date now. Will sort out readme soon.


<!----------------------------------------------------------------------------->

[Badge License]: https://img.shields.io/badge/License-GPL_v2-blue.svg

[License]: LICENSE
