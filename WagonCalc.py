def GetData():
    """
    Функция отвечает за ввод данных данных, необходимых для вычислений
    """

    WagonParams = [24.5, 75.5, 4, 1, 0, 1, 100.0, 90.0]
    # Масса вагона
    # макс загрузка
    # кол-во осей
    # 0/1 авторежим
    # 1/0 дисковый/колодочный
    # 0/1 чугун/композит
    # макс скорость движения порожнего вагона
    # макс скорость груженого вагона]
    BrakeSystemParams = [2, 8, 4, 2]
    # Количество тормозных цилиндров
    # Число колодок (накладок) на вагоне
    # Число тормозных колодок (накладок), на которые действует сила от одного тормозного цилиндра
    # Число тормозных колодок (накладок), воздействующих на одну колесную пару
    BrakeLinkageParams = [5.7, 0.95]
    # Передаточное отношение тормозной рычажной передачи
    # Коэффициент полезного действия тормозной рычажной передачи
    BrakeCylinderParams = [0.254, 0.98, 0.883, 2.3, 0.025, 0.065]
    # Диаметр тормозного цилиндра, м
    # Коэффициент полезного действия тормозного цилиндра
    # Сила предварительного сжатия внутренней отпускной пружины тормозного цилиндра, кН
    # Жесткость отпускной пружины тормозного цилиндра, кН/м
    # Выход штока тормозного цилиндра, м:
    # - минимальное значение
    # - максимальное значение
    BrakeLinkageAutoRegParams = [0.883, 20.8, 0.01, 0.47]
    # Сила предварительного сжатия пружины автоматического регулятора тормозной рычажной передачи, кН
    # Жесткость пружины автоматического регулятора тормозной рычажной передачи, кН/м
    # Величина сжатия пружины автоматического регулятора тормозной рычажной передачи при торможении, м
    # Передаточное число привода автоматического регулятора тормозной рычажной передачи
    ParkingBrakeParams = [100.0, 0.2, 0.087, 72.0, 1.2]
    # Нормативный момент силы, прикладываемый к штурвалу стояночного тормоза, Н • м
    # Коэффициент полезного действия стояночного тормоза
    # Среднее расстояние от точки крепления тяги до оси червячного сектора, м
    # Передаточное отношение червячной передачи
    # Передаточное число рычажной передачи стояночного тормоза от червячного сектора до штока тормозного цилиндра
    BrakeCylinderPressure = [130.0, 300.0, 160.0, 340.0]
    # Давление в тормозном цилиндре, кПа:
    # 1) минимальное давление:
    # - порожний вагон
    # - груженый вагон
    # 2) максимальное давление:
    # - порожний вагон
    # - груженый вагон
    FileName = 'Result'
    # Имя файла, в который будет сохранен результат расчета

    FileName = input('Введите имя файла-отчета: ')
    
    WagonParams[0] = float(input('Введите массу конструкции вагона (в тоннах): '))
    WagonParams[1] = float(input('Введите массу груза (в тоннах): '))
    WagonParams[6] = float(input('Укажите максимальную скорость движения порожнего вагона в км/ч: '))
    WagonParams[7] = float(input('Укажите максимальную скорость движения груженого вагона в км/ч: '))

    BrakeSystemParams[0] = int(input('Введите количество тормозных цилиндров: '))
    BrakeSystemParams[1] = int(input('Введите число колодок (накладок) на вагоне: '))
    BrakeSystemParams[2] = int(input('Введите число тормозных колодок (накладок), на которые действует сила от одного тормозного цилиндра: '))
    BrakeSystemParams[3] = int(input('Введите число тормозных колодок (накладок), воздействующих на одну колесную пару: '))
    
    BrakeLinkageParams[0] = float(input('Введите передаточное отношение тормозной рычажной передачи: '))
    BrakeLinkageParams[1] = float(input('Введите коэффициент полезного действия тормозной рычажной передачи: '))

    BrakeCylinderParams[0] = float(input('Введите диаметр тормозного цилиндра в метрах: '))
    BrakeCylinderParams[1] = float(input('Введите коэффициент полезного действия тормозного цилиндра: '))
    BrakeCylinderParams[2] = float(input('Введите силу предварительного сжатия внутренней отпускной пружины тормозного цилиндра в кН: '))
    BrakeCylinderParams[3] = float(input('Введите жесткость отпускной пружины тормозного цилиндра в кН/м: '))
    BrakeCylinderParams[4] = float(input('Введите минимальный выход штока тормозного цилиндра в м: '))
    BrakeCylinderParams[5] = float(input('Введите максимальный выход штока тормозного цилиндра в м: '))

    BrakeCylinderPressure[0] = float(input('Введите минимальное давление в тормозном цилиндре порожнего вагона, в кПа: '))
    BrakeCylinderPressure[1] = float(input('Введите минимальное давление в тормозном цилиндре груженого вагона, в кПа: '))
    BrakeCylinderPressure[2] = float(input('Введите максимальное давление в тормозном цилиндре порожнего вагона, в кПа: '))
    BrakeCylinderPressure[3] = float(input('Введите ммаксимальное давление в тормозном цилиндре груженого вагона, в кПа: '))
    
    return [WagonParams, BrakeSystemParams, BrakeLinkageParams, BrakeCylinderParams,
            BrakeLinkageAutoRegParams, ParkingBrakeParams, BrakeCylinderPressure, FileName]

def Calculation(inputData):
    """
    Функция отвечает за расчет выходных параметров тормозной системы
    """
    from math import pi, sin
    
    ActualPressingForce = []
    # Действительная сила нажатия тормозных колодок
    
    ActualPressingForce.append((pi*(float(inputData[3][0])**2)*float(inputData[6][0])*inputData[3][1]/4 - (inputData[3][2] +
                                inputData[3][3]*inputData[3][5]) - inputData[4][3]*(inputData[4][0] + inputData[4][1]*inputData[4][2])) *
                                inputData[2][0]*inputData[2][1]/inputData[1][2])
    ActualPressingForce.append((pi*(float(inputData[3][0])**2)*float(inputData[6][2])*inputData[3][1]/4 - (inputData[3][2] + 
                                inputData[3][3]*inputData[3][4]) - 0)*inputData[2][0]*inputData[2][1]/inputData[1][2])
    ActualPressingForce.append((pi*(float(inputData[3][0])**2)*float(inputData[6][1])*inputData[3][1]/4 - (inputData[3][2] + 
                                inputData[3][3]*inputData[3][5]) - inputData[4][3]*(inputData[4][0] + inputData[4][1]*inputData[4][2])) *
                                inputData[2][0]*inputData[2][1]/inputData[1][2])
    ActualPressingForce.append((pi*(float(inputData[3][0])**2)*float(inputData[6][3])*inputData[3][1]/4 - (inputData[3][2] + 
                                inputData[3][3]*inputData[3][4]) - 0)*inputData[2][0]*inputData[2][1]/inputData[1][2])
    
    BrakingDistance = []
    for num in range(4):
        Time = 0.0
        # Время движения вагона
        Speed = 100.0
        # Начальная скорость движения вагона
    
        if num > 1 :
            Speed = inputData[0][7]
        else:
            Speed = inputData[0][6]
    
        actForce = 0.0
        # начальное значение силы нажатия колодок

        if inputData[0][5] == 1:
            FrictionСoef = 0.44 * (0.1*actForce + 20) * (Speed + 150) / (0.4*actForce + 20) / (2*Speed + 150)
            # Действительный коэффициент трения для композиционных тормозных колодок

        SpecificBrakingForce = 0.0
        # Начальная удельная тормозная сила

        SpecificMovementResistance = 0.0
        # Основное удельное сопротивление движению для 4х-осного вагона
        if num > 1 :
            SpecificMovementResistance = 5.2 + (34.2 + 0.732*Speed + 0.022*(Speed**2)) / ((inputData[0][0] + inputData[0][1])/inputData[0][2])
        else:
            SpecificMovementResistance = 5.2 + (34.2 + 0.732*Speed + 0.022*(Speed**2)) / inputData[0][0] / inputData[0][2]
    
        SpeedChange = 12.2 * (SpecificBrakingForce + SpecificMovementResistance) * 1.0 / 3600.0
        # Изменение скорости (приращение)
    
        AverageSpeed = Speed
        # Средняя скорость движения
    
        BrakingDistancesChange = 0.0
        # Приращение тормозного пути

        BrakingDistances = 0.0
        # Тормозной путь

        BrakingDistance0 = []
        # Выходная последовательность данных
        BrakingDistance0.append([round(Time, 1), round(actForce, 3),
                                round(FrictionСoef, 5), round(SpecificBrakingForce, 2),
                                round(SpecificMovementResistance, 3),
                                round(SpeedChange, 4), round(Speed, 3),
                                round(AverageSpeed, 3), round(BrakingDistancesChange, 3),
                                round(BrakingDistances, 2)])
        # Стартовое значение выходной последовательности

        while Speed - SpeedChange > 0 :
            # Промежуточные значения выходной последовательности
            Time = round(Time + 1.0, 0)

            if Time <= 2.0: actForce = 0
            elif Time < 20.0: actForce = ActualPressingForce[num]*sin(5*(Time - 2)*pi/180)
            else: actForce = ActualPressingForce[num]

            if inputData[0][5] == 1:
                FrictionСoef = 0.44 * (0.1*actForce + 20.0) * (Speed + 150.0) / (0.4*actForce + 20.0) / (2*Speed + 150.0)

            if inputData[0][4] == 0:
                if num>1: SpecificBrakingForce = (1000.0*inputData[1][1]*actForce*FrictionСoef) / (inputData[0][0] + inputData[0][1])
                else: SpecificBrakingForce = (1000.0*inputData[1][1]*actForce*FrictionСoef) / inputData[0][0]

            if inputData[0][2] == 4:
                if num>1: SpecificMovementResistance = 5.2 + (34.2 + 0.732*Speed + 0.022*Speed*Speed) / ((inputData[0][0] + inputData[0][1])/
                                                inputData[0][2])
                else: SpecificMovementResistance = 5.2 + (34.2 + 0.732*Speed + 0.022*Speed*Speed) / inputData[0][0] / inputData[0][2]

            AverageSpeed = Speed - 0.5*SpeedChange
            SpeedChange = 12.2 * (SpecificBrakingForce + SpecificMovementResistance) * 1.0 / 3600
            Speed = Speed - SpeedChange

            BrakingDistancesChange = 1*AverageSpeed/3.6
            BrakingDistances = BrakingDistances + BrakingDistancesChange

            BrakingDistance0.append([round(Time, 1), round(actForce, 3),
                                round(FrictionСoef, 5), round(SpecificBrakingForce, 2),
                                round(SpecificMovementResistance, 3),
                                round(SpeedChange, 4), round(Speed, 3),
                                round(AverageSpeed, 3), round(BrakingDistancesChange, 3),
                                round(BrakingDistances, 2)])
        else:
            # Последние значения выходной последовательности
            Speed = 0.0
            AverageSpeed = 0.0
            Time = round(Time + 1.0, 0)

            if Time <= 2.0: actForce = 0
            elif Time < 20.0: actForce = ActualPressingForce[num]*sin(5*(Time - 2)*pi/180)
            else: actForce = ActualPressingForce[num]

            if inputData[0][5] == 1:
                FrictionСoef = 0.44 * (0.1*actForce + 20) * (Speed + 150) / (0.4*actForce + 20) / (2*Speed + 150)

            if inputData[0][4] == 0:
                if num>1: SpecificBrakingForce = (1000.0*inputData[1][1]*actForce*FrictionСoef) / (inputData[0][0] + inputData[0][1])
                else: SpecificBrakingForce = (1000.0*inputData[1][1]*actForce*FrictionСoef) / inputData[0][0]

            if inputData[0][2] == 4:
                if num>1: SpecificMovementResistance = 5.2 + (34.2 + 0.732*Speed + 0.022*Speed*Speed) / ((inputData[0][0] + 
                                                            inputData[0][1])/inputData[0][2])
                else: SpecificMovementResistance = 5.2 + (34.2 + 0.732*Speed + 0.022*Speed*Speed) / inputData[0][0] / inputData[0][2]

            SpeedChange = 12.2 * (SpecificBrakingForce + SpecificMovementResistance) * 1.0 / 3600

            BrakingDistance0.append([round(Time, 1), round(actForce, 3),
                          round(FrictionСoef, 5), round(SpecificBrakingForce, 2),
                          round(SpecificMovementResistance, 3),
                          round(SpeedChange, 4), round(Speed, 3),
                          round(AverageSpeed, 3), round(BrakingDistancesChange, 3),
                          round(BrakingDistances, 2)])
            
            BrakingDistance.append(BrakingDistance0)
    
    PressingForceCoefficient = []
    #Сила нажатия и ее коэффициент
    PressingForceCoefficient.append(1.22*float(ActualPressingForce[0])*(0.1*float(ActualPressingForce[0]) + 20.0)/(0.4*float(ActualPressingForce[0]) + 20.0))
    PressingForceCoefficient.append(1.22*float(ActualPressingForce[2])*(0.1*float(ActualPressingForce[2]) + 20.0)/(0.4*float(ActualPressingForce[2]) + 20.0))
    PressingForceCoefficient.append(float(inputData[1][1])*float(PressingForceCoefficient[0])/9.81/float(inputData[0][0]))
    PressingForceCoefficient.append(float(inputData[1][1])*float(PressingForceCoefficient[1])/9.81/(float(inputData[0][0])+float(inputData[0][1])))

    WheelSkid = []
    # Параметры для определения юза
    for num in range(2):
        WheelSkid0 = []
        for i in range(5):
            Speed = round((i+1) * float(inputData[0][6+num]/5))
            actForce = round(ActualPressingForce[1 + 2*num], 2)
            FrictionСoef = round(0.44 * (0.1*actForce + 20) * (Speed + 150) / (0.4*actForce + 20) / (2*Speed + 150), 3)
            AxialLoad = round(0.17 - 0.0015*((inputData[0][0] + num*inputData[0][1])/inputData[0][2] - 5), 3)
            SpeedFunc = round((Speed + 81)/(2.5*Speed + 85.3), 3)
            Adhesion = round(AxialLoad*SpeedFunc, 3)
            SumForce = round(1000 * 9.81 * Adhesion)
            SpecificBrakingForce = round((1000.0*inputData[1][1]*actForce*FrictionСoef) / (inputData[0][0] + num*inputData[0][1]))
            if SpecificBrakingForce<SumForce:
                Skid = 'Качение'
            else:
                Skid = 'Юз'
            WheelSkid0.append([Speed, actForce, FrictionСoef, AxialLoad, SpeedFunc, Adhesion, SumForce, SpecificBrakingForce, Skid])
        WheelSkid.append(WheelSkid0)
        
    Power = round(pow(inputData[0][7], 3)*(inputData[0][0] + inputData[0][1])/inputData[0][2]/186.6/BrakingDistance[3][len(BrakingDistance[3])-1][9]/inputData[1][3], 1)
    # Мощность

    ParkingBrake = []
    #Параметры стояночного тормоза
    ParkingBrake.append(round(inputData[2][0]*inputData[2][1]*(inputData[5][0]*inputData[5][3]*inputData[5][4]*inputData[5][1]/1000/inputData[5][2] - (inputData[3][2] +
                                inputData[3][3]*inputData[3][5]) - inputData[4][3]*(inputData[4][0] + inputData[4][1]*inputData[4][2]))/inputData[1][2], 2))
    ParkingBrake.append(round(0.44*(0.1*ParkingBrake[0] + 20)/(0.4*ParkingBrake[0] + 20), 2))
    ParkingBrake.append(round(1000*ParkingBrake[0]*inputData[1][2]*ParkingBrake[1]*1/9.81/(inputData[0][0] + inputData[0][1])))

    return [inputData, ActualPressingForce, BrakingDistance, PressingForceCoefficient, WheelSkid, Power, ParkingBrake]
    
def ReportGen(inputData):
    """
    Функция импорта всех данных в Excel-файл
    """
    TableNames = ['Таблица 1 — Общие характеристики вагона',
                  'Таблица 2 — Общие характеристики тормозной системы',
                  'Таблица 3 — Характеристики тормозной рычажной передачи',
                  'Таблица 4 — Характеристики тормозного цилиндра',
                  'Таблица 5 — Характеристики автоматического регулятора тормозной рычажной передачи',
                  'Таблица 6 — Характеристики стояночного тормоза',
                  'Таблица 7 - Расчетное давление в тормозном цилиндре',
                  'Таблица 8 - Результаты расчета действительной силы нажатия тормозных колодок',
                  'Таблица 9 - Результаты расчета тормозного пути',
                  'Таблица 9.1 - Результаты расчета тормозного пути для порожнего вагона с минимальной действительной силой нажатия тормозных колодок со скорости ' + str(inputData[0][0][6]) + ' км/ч',
                  'Таблица 9.2 - Результаты расчета тормозного пути для порожнего вагона с максимальной действительной силой нажатия тормозных колодок со скорости ' + str(inputData[0][0][6]) + ' км/ч',
                  'Таблица 9.3 - Результаты расчета тормозного пути для груженого вагона с минимальной действительной силой нажатия тормозных колодок со скорости ' + str(inputData[0][0][7]) + ' км/ч',
                  'Таблица 9.4 - Результаты расчета тормозного пути для груженого вагона с максимальной действительной силой нажатия тормозных колодок со скорости ' + str(inputData[0][0][7]) + ' км/ч',
                  'Таблица 10 - Результаты расчета коэффициента силы нажатия тормозных колодок',
                  'Таблица 11 - Результаты проверки отсутствия юза колес порожнего вагона при экстренном торможении',
                  'Таблица 12 - Результаты проверки отсутствия юза колес груженого вагона с минимальной массой при экстренном торможении',
                  'Таблица 13 - Результаты расчета стояночного тормоза']
    ColumnNames = ['Параметр', 'Значение',
                   'Наименование', 'Значение, кН', 'порожний вагон', 'груженый вагон',
                   'Тормозной путь', 'Значение, м', 'Время торможения, с',
                   't, c', 'Kд, кН', 'fi_к/д', 'bm, Н/т', 'Wox, Н/т', 'dV, км/ч', 'V, км/ч', 'Vср, км/ч', 'dSт, м', 'Sт, м',  #9
                   'Наименование рассчитанного показателя', 'Значение', 'порожний вагон', 'груженый вагон',  #19
                   'V, км/ч', 'Kд, кН', 'fi_к/д', 'psi(q0)', 'psi(V)', '[psi_р]', '1000g[psi_р]', 'bm, Н/т', 'Состояние',
                   'Действительная сила нажатия тормозных колодок от действия стояночного тормоза Кдс, кН',  #32
                   'Действительный статический коэффициент трения fi_кс',
                   'Расчетный уклон пути i, на котором удерживается вагон стояночным тормозом, %о']
    RowNames = ['Тара вагона, т',
                'Максимальная загрузка вагона, т',
                'Кол-во осей вагона',
                'Наличие авторежима',
                'Тип тормоза',
                'Тип тормозных колодок',
                'Максимальная скорость движения, км/ч:',
                '  - порожнего вагона',
                '  - груженого вагона',
                'Количество тормозных цилиндров',
                'Число колодок (накладок) на вагоне',
                'Число тормозных колодок (накладок), на которые действует сила от одного тормозного цилиндра',
                'Число тормозных колодок (накладок), воздействующих на одну колесную пару',
                'Передаточное отношение тормозной рычажной передачи',
                'Коэффициент полезного действия тормозной рычажной передачи',
                'Диаметр тормозного цилиндра, м',
                'Коэффициент полезного действия тормозного цилиндра',
                'Сила предварительного сжатия внутренней отпускной пружины тормозного цилиндра, кН',
                'Жесткость отпускной пружины тормозного цилиндра, кН/м',
                'Выход штока тормозного цилиндра, м:',
                '  - минимальное значение',
                '  - максимальное значение',
                'Сила предварительного сжатия пружины автоматического регулятора тормозной рычажной передачи, кН',
                'Жесткость пружины автоматического регулятора тормозной рычажной передачи, кН/м',
                'Величина сжатия пружины автоматического регулятора тормозной рычажной передачи при торможении, м',
                'Передаточное число привода автоматического регулятора тормозной рычажной передачи',
                'Нормативный момент силы, прикладываемый к штурвалу стояночного тормоза, Н•м',
                'Коэффициент полезного действия стояночного тормоза',
                'Среднее расстояние от точки крепления тяги до оси червячного сектора, м',
                'Передаточное отношение червячной передачи',
                'Передаточное число рычажной передачи стояночного тормоза от червячного сектора до штока тормозного цилиндра',
                'Давление в тормозном цилиндре, кПа:',
                '1) минимальное давление:',
                '  - порожний вагон',
                '  - груженый вагон',
                '2) максимальное давление:',
                'Минимальная действительная сила нажатия тормозных колодок:',
                'Максимальная действительная сила нажатия тормозных колодок:',
                'Для порожнего вагона с минимальной действительной силой нажатия тормозных колодок с максимальной скорости',
                'Для порожнего вагона с максимальной действительной силой нажатия тормозных колодок с максимальной скорости',
                'Для груженого вагона с минимальной действительной силой нажатия тормозных колодок с максимальной скорости',
                'Для груженого вагона с максимальной действительной силой нажатия тормозных колодок с максимальной скорости',
                'Минимальная расчетная сила нажатия тормозных колодок, кН',
                'Расчетный коэффициент силы нажатия тормозных колодок',
                'Значение средней мощности N, кВт, приходящейся на одну колодку (накладку) при экстренном торможении груженого вагона']
    TabNames = ['Входные параметры', 'Торм.путь', 'Эффективность']

    import xlsxwriter
    workbook = xlsxwriter.Workbook(inputData[0][7]+'.xlsx') 
    worksheet = workbook.add_worksheet(TabNames[0])

    Title = workbook.add_format(
        {
            'font_name': 'Times New Roman',
            'font_size': 14,
            'italic': True,
            'text_wrap': True

        }
    )

    Heading = workbook.add_format(
        {
            'font_name': 'Times New Roman',
            'font_size': 14,
            'bold': True,
            'align': 'center',
            'text_wrap': True
        }
    )

    CommonL = workbook.add_format(
        {
            'font_name': 'Times New Roman',
            'font_size': 12,
            'align': 'left',
            'text_wrap': True
        }
    )

    CommonC = workbook.add_format(
        {
            'font_name': 'Times New Roman',
            'font_size': 12,
            'valign': 'center'
        }
    )

    worksheet.set_column("A:A", 60)
    worksheet.set_column("B:B", 20)

    worksheet.merge_range('A1:B1', TableNames[0], Title)
    worksheet.write('A2', ColumnNames[0], Heading)
    worksheet.write('B2', ColumnNames[1], Heading)
    worksheet.write('A3', RowNames[0], CommonL)
    worksheet.write('B3', inputData[0][0][0], CommonC)
    worksheet.write('A4', RowNames[1], CommonL)
    worksheet.write('B4', inputData[0][0][1], CommonC)
    worksheet.write('A5', RowNames[2], CommonL)
    worksheet.write('B5', str(inputData[0][0][2]) + '-осный вагон', CommonC)
    worksheet.write('A6', RowNames[3], CommonL)
    if inputData[0][0][3] == 1:
        worksheet.write('B6', 'есть', CommonC)
    else:
        worksheet.write('B6', 'нет', CommonC)
    worksheet.write('A7', RowNames[4], CommonL)
    if inputData[0][0][4] == 0:
        worksheet.write('B7', 'колодочный', CommonC)
    else:
        worksheet.write('B7', 'дисковый', CommonC)
    worksheet.write('A8', RowNames[5], CommonL)
    if inputData[0][0][5] == 0:
        worksheet.write('B8', 'чугунные', CommonC)
    else:
        worksheet.write('B8', 'композиционные', CommonC)
    worksheet.write('A9', RowNames[6], CommonL)
    worksheet.write('A10', RowNames[7], CommonL)
    worksheet.write('B10', inputData[0][0][6], CommonC)
    worksheet.write('A11', RowNames[8], CommonL)
    worksheet.write('B11', inputData[0][0][7], CommonC)
    
    worksheet.merge_range('A13:B13', TableNames[1], Title)
    worksheet.write('A14', ColumnNames[0], Heading)
    worksheet.write('B14', ColumnNames[1], Heading)
    worksheet.write('A15', RowNames[9], CommonL)
    worksheet.write('B15', inputData[0][1][0], CommonC)
    worksheet.write('A16', RowNames[10], CommonL)
    worksheet.write('B16', inputData[0][1][1], CommonC)
    worksheet.write('A17', RowNames[11], CommonL)
    worksheet.write('B17', inputData[0][1][2], CommonC)
    worksheet.write('A18', RowNames[12], CommonL)
    worksheet.write('B18', inputData[0][1][3], CommonC)

    worksheet.merge_range('A20:B20', TableNames[2], Title)
    worksheet.write('A21', ColumnNames[0], Heading)
    worksheet.write('B21', ColumnNames[1], Heading)
    worksheet.write('A22', RowNames[13], CommonL)
    worksheet.write('B22', inputData[0][2][0], CommonC)
    worksheet.write('A23', RowNames[14], CommonL)
    worksheet.write('B23', inputData[0][2][1], CommonC)

    worksheet.merge_range('A25:B25', TableNames[3], Title)
    worksheet.write('A26', ColumnNames[0], Heading)
    worksheet.write('B26', ColumnNames[1], Heading)
    worksheet.write('A27', RowNames[15], CommonL)
    worksheet.write('B27', inputData[0][3][0], CommonC)
    worksheet.write('A28', RowNames[16], CommonL)
    worksheet.write('B28', inputData[0][3][1], CommonC)
    worksheet.write('A29', RowNames[17], CommonL)
    worksheet.write('B29', inputData[0][3][2], CommonC)
    worksheet.write('A30', RowNames[18], CommonL)
    worksheet.write('B30', inputData[0][3][3], CommonC)
    worksheet.write('A31', RowNames[19], CommonL)
    worksheet.write('A32', RowNames[20], CommonL)
    worksheet.write('B32', inputData[0][3][4], CommonC)
    worksheet.write('A33', RowNames[21], CommonL)
    worksheet.write('B33', inputData[0][3][5], CommonC)

    worksheet.merge_range('A35:B37', TableNames[4], Title)
    worksheet.write('A38', ColumnNames[0], Heading)
    worksheet.write('B38', ColumnNames[1], Heading)
    worksheet.write('A39', RowNames[22], CommonL)
    worksheet.write('B39', inputData[0][4][0], CommonC)
    worksheet.write('A40', RowNames[23], CommonL)
    worksheet.write('B40', inputData[0][4][1], CommonC)
    worksheet.write('A41', RowNames[24], CommonL)
    worksheet.write('B41', inputData[0][4][2], CommonC)
    worksheet.write('A42', RowNames[25], CommonL)
    worksheet.write('B42', inputData[0][4][3], CommonC)

    worksheet.merge_range('A44:B44', TableNames[5], Title)
    worksheet.write('A45', ColumnNames[0], Heading)
    worksheet.write('B45', ColumnNames[1], Heading)
    worksheet.write('A46', RowNames[26], CommonL)
    worksheet.write('B46', inputData[0][5][0], CommonC)
    worksheet.write('A47', RowNames[27], CommonL)
    worksheet.write('B47', inputData[0][5][1], CommonC)
    worksheet.write('A48', RowNames[28], CommonL)
    worksheet.write('B48', inputData[0][5][2], CommonC)
    worksheet.write('A49', RowNames[29], CommonL)
    worksheet.write('B49', inputData[0][5][3], CommonC)
    worksheet.write('A50', RowNames[30], CommonL)
    worksheet.write('B50', inputData[0][5][4], CommonC)

    worksheet.merge_range('A52:B52', TableNames[6], Title)
    worksheet.write('A53', ColumnNames[0], Heading)
    worksheet.write('B53', ColumnNames[1], Heading) 
    worksheet.write('A54', RowNames[31], CommonL)
    worksheet.write('A55', RowNames[32], CommonL)
    worksheet.write('A56', RowNames[33], CommonL)
    worksheet.write('B56', inputData[0][6][0], CommonC)
    worksheet.write('A57', RowNames[34], CommonL)
    worksheet.write('B57', inputData[0][6][1], CommonC)
    worksheet.write('A58', RowNames[35], CommonL)
    worksheet.write('A59', RowNames[33], CommonL)
    worksheet.write('B59', inputData[0][6][2], CommonC)
    worksheet.write('A60', RowNames[34], CommonL)
    worksheet.write('B60', inputData[0][6][3], CommonC)

    worksheet = workbook.add_worksheet(TabNames[1])
    worksheet.set_column("A:AQ", 13)

    worksheet.merge_range('A1:J1', TableNames[7], Title)
    worksheet.merge_range('A2:F3', ColumnNames[2], Heading)
    worksheet.merge_range('G2:J2', ColumnNames[3], Heading)
    worksheet.merge_range('G3:H3', ColumnNames[4], Heading)
    worksheet.merge_range('I3:J3', ColumnNames[5], Heading)
    worksheet.merge_range('A4:F5', RowNames[36], CommonL)
    worksheet.merge_range('A6:F7', RowNames[37], CommonL)
    worksheet.merge_range('G4:H5', round(inputData[1][0], 2), CommonC)
    worksheet.merge_range('I4:J5', round(inputData[1][2], 2), CommonC)
    worksheet.merge_range('G6:H7', round(inputData[1][1], 2), CommonC)
    worksheet.merge_range('I6:J7', round(inputData[1][3], 2), CommonC)
    
    worksheet.merge_range('A9:J9', TableNames[8], Title)
    worksheet.merge_range('A10:F10', ColumnNames[6], Heading)
    worksheet.merge_range('G10:H10', ColumnNames[7], Heading)
    worksheet.merge_range('I10:J10', ColumnNames[8], Heading)
    worksheet.merge_range('A11:F12', RowNames[38], CommonL)
    worksheet.merge_range('A13:F14', RowNames[39], CommonL)
    worksheet.merge_range('A15:F16', RowNames[40], CommonL)
    worksheet.merge_range('A17:F18', RowNames[41], CommonL)
    worksheet.merge_range('G11:H12', str(int(inputData[2][0][len(inputData[2][0])-1][9])), CommonC)
    worksheet.merge_range('I11:J12', str(int(inputData[2][0][len(inputData[2][0])-1][0])), CommonC)
    worksheet.merge_range('G13:H14', str(int(inputData[2][1][len(inputData[2][1])-1][9])), CommonC)
    worksheet.merge_range('I13:J14', str(int(inputData[2][1][len(inputData[2][1])-1][0])), CommonC)
    worksheet.merge_range('G15:H16', str(int(inputData[2][2][len(inputData[2][2])-1][9])), CommonC)
    worksheet.merge_range('I15:J16', str(int(inputData[2][2][len(inputData[2][2])-1][0])), CommonC)
    worksheet.merge_range('G17:H18', str(int(inputData[2][3][len(inputData[2][3])-1][9])), CommonC)
    worksheet.merge_range('I17:J18', str(int(inputData[2][3][len(inputData[2][3])-1][0])), CommonC)

    for c in range(len(inputData[2])):
        worksheet.merge_range(19, c*11, 21, c*11+9, TableNames[c+9], Title)
        for a in range(9, 19, 1):
            worksheet.write(22, a-9 + c*11, ColumnNames[a], Heading)
        for i in range(len(inputData[2][c])):
            for j in range(len(inputData[2][c][i])):
                worksheet.write(i+23, j + c*11, inputData[2][c][i][j])

    worksheet = workbook.add_worksheet(TabNames[2])
    worksheet.set_column("A:AQ", 14)
    
    worksheet.merge_range('A1:J1', TableNames[13], Title)
    worksheet.merge_range('A2:F3', ColumnNames[19], Heading)
    worksheet.merge_range('G2:J2', ColumnNames[20], Heading)
    worksheet.merge_range('G3:H3', ColumnNames[21], Heading)
    worksheet.merge_range('I3:J3', ColumnNames[22], Heading)
    worksheet.merge_range('A4:F5', RowNames[42], CommonL)
    worksheet.merge_range('G4:H5', round(inputData[3][0], 2), CommonC)
    worksheet.merge_range('I4:J5', round(inputData[3][1], 2), CommonC)
    worksheet.merge_range('A6:F7', RowNames[43], CommonL)
    worksheet.merge_range('G6:H7', round(inputData[3][2], 3), CommonC)
    worksheet.merge_range('I6:J7', round(inputData[3][3], 3), CommonC)

    worksheet.merge_range('A9:J9', TableNames[14], Title)
    for i in range(9):
        worksheet.write(9, i, ColumnNames[23+i], Heading)
    for i in range(5):
        for j in range(9):
            worksheet.write(10+i, j, inputData[4][0][i][j], CommonC)

    worksheet.merge_range('A17:J17', TableNames[15], Title)
    for i in range(9):
        worksheet.write(17, i, ColumnNames[23+i], Heading)
    for i in range(5):
        for j in range(9):
            worksheet.write(18+i, j, inputData[4][1][i][j], CommonC)

    worksheet.merge_range('A25:H26', RowNames[44], CommonL)
    worksheet.merge_range('I25:I26', inputData[5], CommonC)

    worksheet.merge_range('A28:J28', TableNames[16], Title)
    worksheet.merge_range('A29:C32', ColumnNames[32], Heading)
    worksheet.merge_range('D29:F32', ColumnNames[33], Heading)
    worksheet.merge_range('G29:I32', ColumnNames[34], Heading)
    worksheet.merge_range('A33:C33', inputData[6][0], CommonC)
    worksheet.merge_range('D33:F33', inputData[6][1], CommonC)
    worksheet.merge_range('G33:I33', inputData[6][2], CommonC)
    workbook.close()

ReportGen(Calculation(GetData()))