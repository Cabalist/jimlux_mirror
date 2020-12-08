Attribute VB_Name = "ExponentialIntegral"
' exponential integrals
'Taken from Numerical Recipes in Fortran, pp.250-252

Const Euler = 0.57721566
Const PI = 3.14159265358979
Const FPMIN = 1E-30
Const TMIN = 2
Const MAXIT = 100
Const EPS = 0.00000006

Sub cisi(X, ci, si)


Dim h As Complex, b As Complex, c As Complex, d As Complex, del As Complex
Dim odd As Boolean

t = Abs(X)
If t = 0 Then       'special case
    si = 0
    ci = -1 / FPMIN
    Exit Sub
End If
If t > TMIN Then
    b = cmplx(1, t)
    c = cmplx(1 / FPMIN, 0)
    d = cinverse(b)
    h = d
    For i = 2 To MAXIT
        a = -(i - 1) ^ 2
        b.r = b.r + 2
        d = cinverse(cadd(cscale(d, a), b))
        c = cadd(b, cscale(cinverse(c), a))
        del = cmult(c, d)
        h = cmult(h, del)
        If (Abs(del.r - 1) + Abs(del.i)) < EPS Then Exit For
        
        
    Next i
    If (Abs(del.r - 1) + Abs(del.i)) > EPS Then Err.Raise 0, "CF failed in CISI"
    h = cmult(cmplx(Cos(t), -Sin(t)), h)
    ci = -h.r
    si = PI / 2 + h.i
    
Else
    If t < Sqr(FPMIN) Then
        sumc = 0
        sums = t
    Else
        Sum = 0
        sums = 0
        sumc = 0
        sign = 1
        fact = 1
        odd = True
        For k = 1 To MAXIT
            fact = fact * t / k
            term = fact / k
            Sum = Sum + sign * term
            er = term / Abs(Sum)
            If odd Then
                sign = -sign
                sums = Sum
                Sum = sumc
            Else
                sumc = Sum
                Sum = sums
            End If
            If er < EPS Then Exit For
            odd = Not odd
        Next k
        If er > EPS Then Err.Raise 0, "MAXIT exceeded in cisi"
    End If
    si = sums
    ci = sumc + Log(t) + Euler
End If
If X < 0 Then si = -si


    

End Sub

