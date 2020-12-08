Attribute VB_Name = "ComplexMath"
Type Complex
    r As Double
    i As Double
End Type
Function cinverse(a As Complex) As Complex
m = a.r * a.r + a.i * a.i
cinverse.r = a.r / m
cinverse.i = -a.i / m
End Function
Function cadd(a As Complex, b As Complex) As Complex
cadd.r = a.r + b.r
cadd.i = a.i + b.i
End Function
Function csub(a As Complex, b As Complex) As Complex
csub.r = a.r - b.r
csub.i = a.i - b.i
End Function
Function conjugate(a As Complex) As Complex
conjugate.r = a.r
conjugate.i = -a.i

End Function

Function cmult(a As Complex, b As Complex) As Complex
cmult.r = a.r * b.r - a.i * b.i
cmult.i = a.r * b.i + a.i * b.r

End Function
Function cscale(a As Complex, s) As Complex
cscale.r = a.r * s
cscale.i = a.i * s
End Function
Function creal(a As Complex) As Double
creal = a.r

End Function
Function cimag(a As Complex) As Double
cimag = a.i
End Function
Function cmodulus(a As Complex) As Double
cmodulus = a.r * a.r + a.i * a.i
End Function
Function cdiv(a As Complex, b As Complex) As Complex

denom = cmodulus(b)

cdiv = cscale(cmult(a, conjugate(b)), 1 / denom)


End Function
Function cmplx(a, b) As Complex
cmplx.r = a
cmplx.i = b
End Function
Function cexp(x As Complex) As Complex

er = Exp(x.r)
cexp.r = er * Cos(x.i)
cexp.i = er * Sin(x.i)

End Function
