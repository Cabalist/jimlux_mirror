Attribute VB_Name = "hyperbolic"
' complex hyperbolic functions

Function csinh(x As Complex) As Complex
csinh.r = 0.5 * (Exp(x.r) - Exp(-x.r)) * Cos(x.i)
csinh.i = 0.5 * (Exp(x.r) + Exp(-x.r)) * Sin(x.i)

End Function
Function ccosh(x As Complex) As Complex
ccosh.r = 0.5 * (Exp(x.r) + Exp(-x.r)) * Cos(x.i)
ccosh.i = 0.5 * (Exp(x.r) - Exp(-x.r)) * Sin(x.i)

End Function
Sub chboth(x As Complex, sinh As Complex, cosh As Complex)
' slightly faster to compute both at once
c = Cos(x.i)
s = Sin(x.i)
e1 = Exp(x.r)
e2 = 1 / e1
sinh.r = 0.5 * (e1 - e2) * c
sinh.i = 0.5 * (e1 + e2) * s
cosh.r = 0.5 * (e1 + e2) * c
cosh.i = 0.5 * (e1 - e2) * s


End Sub
Sub ihboth(x, sinh As Complex, cosh As Complex)
' special case where argument is imaginary
' exp(re(x)) will be 1
c = Cos(x)
s = Sin(x)
sinh.r = 0
sinh.i = s
cosh.r = c
cosh.i = 0
End Sub
'
' real hyperbolic functions
Function sinh(x)
sinh = (Exp(x) - Exp(-x)) / 2
End Function
Function cosh(x)
cosh = (Exp(x) + Exp(-x)) / 2

End Function
Sub hboth(x, sinh, cosh)
e1 = x
e2 = 1 / x
sinh = (e1 - e2) / 2
cosh = (e1 + d2) / 2

End Sub
