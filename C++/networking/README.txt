
NOBLOCKING SOCKET:
Når socket er noBlocking == false så vil recvfrom() holde igjen prosessen til den mottar en pakke. 
Dette skapte problemer når jeg hadde trået ut serveren slik at programmet ikke ville avslutte.
Alternativet er å bruke noBlocking == true, men da må det loopes som vil føre til 100% CPU trå bruk
med mindre man bruker sleep.