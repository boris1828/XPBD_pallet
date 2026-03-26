import xml.etree.ElementTree as ET

# Carica il file XML
tree = ET.parse('c:/Users/sammy/Desktop/Tirocinio/Progetto/A Safe Place To Stay/packaging_simulation/BlenderProject/MoreSchemes/Nuova cartella/C12262_2/Layer/Format n3 - Pounch Heninz by nature UK - 6x1 - Type 2.layer')
root = tree.getroot()

# Inizializza i valori massimi
max_x = float('-inf')
max_y = float('-inf')

# Itera attraverso tutti gli elementi PalSchema_SPDisposalClass
for sp_disposal in root.findall('.//PalSchema_SPDisposalClass'):
    x = int(sp_disposal.find('_x').text)
    y = int(sp_disposal.find('_y').text)
    
    if x > max_x:
        max_x = x
    if y > max_y:
        max_y = y

print(f"Max _x: {max_x}")
print(f"Max _y: {max_y}")