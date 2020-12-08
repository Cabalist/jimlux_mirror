Attribute VB_Name = "TransmissionLine"
' transmission line equations
Function cTL(Zl As Complex, Z0 As Complex, gamma As Complex) As Complex
Dim sg As Complex, cg As Complex

chboth gamma, sg, cg

cTL = cmult(Z0, cdiv(cadd(cmult(Zl, cg), cmult(Z0, sg)), cadd(cmult(Zl, sg), cmult(Z0, cg))))


End Function

' faster version when all impedances are real, and TL is lossless
' L is length in wavelengths
Function losslessTL(Zl, Z0, L)
a = L * 2 * 3.141592635

End Function
