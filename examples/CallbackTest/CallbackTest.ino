
int addFunction(int a, int b) {
    return (a+b); 
}

int subtractFunction(int a, int b) {
    return (a-b); 
}

int executeFunction(int arg1, int arg2, int (*function)(int,int)) {
    return function(arg1,arg2);
}

void setup() {
    Serial.begin(57600);

}

void loop() {
      Serial.println(executeFunction(1,2, addFunction));
      Serial.println(executeFunction(2,2, subtractFunction));
      delay(500);
}
