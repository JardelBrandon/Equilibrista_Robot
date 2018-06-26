int c;
void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  Serial.begin(9600);
}

//Digitar 1 para acender e 2 para desligar!
void loop()
{
  if (Serial.available() > 0)
  {
    c = Serial.read();
    Serial.print(c);
  if (c == 49)
    {
      digitalWrite(13, HIGH);
    }
    if (c == 50)
    {
      digitalWrite(13, LOW);
    }
    if (c == 51)
    {
      Serial.print("Hello World!");
      
    }
    else
    {
      Serial.print("Entrada inválida");
     
     }
  }
}
