import json


#from pprint import pprint

TYPES = {
    "bool": "unsigned char",
    "int8": "signed char",
    "uint8": "unsigned char",
    "int16": "signed short",
    "uint16": "unsigned short",
    "int32": "signed long",
    "uint32": "unsigned long",
    "int64": "signed long long",
    "uint64": "unsigned long long",
    "double": "double",
    "float": "float",
    "string": "char",
}






f = open('commands.h', encoding='utf-8', mode='w')

f.write("#ifndef COMMANDS_H\n")
f.write("#define COMMANDS_H\n\n")



f.write("\n")



with open('SPD_Client.json','r', -1,'utf-8') as data_file:


    
    data = json.load(data_file)

    

    #

    f.write("// Commands:\n\n")

    #f.write("typedef enum {\n")
    

    for c in data["commands"]:
        
        NAME = c["name"]
        ID = c["id"]
        ALIAS = c["alias"]

        f.write("#define CMD_" + ALIAS + " " + str(ID) + " // " + NAME + "\n")

    #f.write("} commands;\n\n")

    f.write("\n")

    f.write("#pragma pack(push,1)\n")

    #



    for c in data["commands"]:
        NAME = c["name"]
        ID = c["id"]
        ALIAS = c["alias"]
        rcv = c["in"]
        snt = c["out"]

        
        f.write("/*\n")
        f.write("    " + ALIAS)
        #f.write(str(ID))
        f.write(": ")
        f.write(NAME)
        f.write("\n*/\n\n")


        # to host

        if rcv:


            f.write("// To host:\n")
            f.write("typedef struct {\n")


            n = 0
            for r in rcv:
                name = r["name"]
                alias = r["alias"]
                tp = TYPES[r["type"].lower()]

            
                f.write("    ")
                f.write(tp)
                f.write(" ")
                #f.write("value_")
                #f.write(str(n))
                f.write(alias)

                if tp == "char": f.write("[62]")
                
                f.write("; // ")
                f.write(name)
                f.write("\n")

                n += 1

        
            f.write("} ")
            #f.write("cmd_")
            #f.write(str(ID))
            f.write(ALIAS + "_TX_t;\n\n")


        # from host

        if snt:

            f.write("// From host:\n")
            f.write("typedef struct {\n")


            n = 0
            for s in snt:
                name = s["name"]
                alias = s["alias"]
                tp = TYPES[s["type"].lower()]

            
                f.write("    ")
                f.write(tp)
                f.write(" ")
                #f.write("value_")
                #f.write(str(n))
                f.write(alias)

                if tp == "char": f.write("[62]")
                
                f.write("; // ")
                f.write(name)
                f.write("\n")

                n += 1

        

        
            f.write("} ")
            #f.write("cmd_")
            #f.write(str(ID))
            #f.write("_from_host_t;\n\n")
            f.write(ALIAS + "_RX_t;\n\n")


        

f.write("#pragma pack(pop)\n\n")
    
f.write("#endif // COMMANDS_H\n")    
f.close()

print("Готово!")
