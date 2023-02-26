package frc.robot;

import java.lang.reflect.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class AutoConfigurable {
    static class dontGrabFrom{
        static List<Field> allFields;
        static HashMap<String, GenericEntry> shuffleBoardElements = new HashMap<String, GenericEntry>(); 
    }
    public AutoConfigurable(){

    }
    public void InitializeShuffleBoard(){
        dontGrabFrom.allFields = getAllVariablesAndInit(null, null);
    }
    public void UpdateVariables(){
        
        for(Field classField: dontGrabFrom.allFields){
            classField.setAccessible(true);
            
            if(classField.getType() == double.class){
                double newValue = dontGrabFrom.shuffleBoardElements.get(classField.getName()).getDouble(3.7);
                try {
                    classField.set(null, newValue);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            else if(classField.getType() == int.class){
                int newValue = (int)(dontGrabFrom.shuffleBoardElements.get(classField.getName()).getDouble(1));
                try {
                    classField.set(null, newValue);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            else if(classField.getType() == float.class){
                float newValue = (float) (dontGrabFrom.shuffleBoardElements.get(classField.getName()).getDouble(3.7));
                try {
                    classField.set(null, newValue);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            else if(classField.getType() == boolean.class){
                boolean newValue = (boolean) (dontGrabFrom.shuffleBoardElements.get(classField.getName()).getBoolean(true));
                System.out.println(newValue);
                try {
                    classField.set(null, newValue);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            
        }
        
    }
    public List<Field> getAllVariablesAndInit(Class<?> getFieldsFrom, List<Field> fieldsSoFar){
        if(fieldsSoFar == null){ fieldsSoFar = new ArrayList<>();}
        if(getFieldsFrom == null){getFieldsFrom = this.getClass();}
        for(Field f: getFieldsFrom.getDeclaredFields()){
            fieldsSoFar.add(f);
            initShuffles(f, getFieldsFrom);
        }
        for(Class<?> c: getFieldsFrom.getDeclaredClasses()){
            if(c.getName() != "dontGrabFrom"){
                getAllVariablesAndInit(c,fieldsSoFar);
            }
        }
        return fieldsSoFar;
    }
    public void initShuffles (Field fieldToInit, Class<?> classFieldIn){
        fieldToInit.setAccessible(true);
        System.out.println(fieldToInit.getName());
        if(fieldToInit.getType() == int.class ||fieldToInit.getType() == float.class ||fieldToInit.getType() == double.class){
            
            dontGrabFrom.shuffleBoardElements.put(fieldToInit.getName(),
                    Shuffleboard.getTab("Configuration")
                        .add(fieldToInit.getName(), 1)
                        .withWidget(BuiltInWidgets.kTextView)
                        .withPosition(1, 1)
                        .withSize(2, 1)
                        .getEntry());
        }
        else if(fieldToInit.getType() == boolean.class){
            System.out.println(fieldToInit.getName());
            dontGrabFrom.shuffleBoardElements.forEach(null);
            dontGrabFrom.shuffleBoardElements.put(fieldToInit.getName(),
            Shuffleboard.getTab("Configuration")
                .add(fieldToInit.getName(), true)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(1, 1)
                .withSize(1, 1)
                .getEntry());
        }
        else{
            System.out.println("The variable:" + fieldToInit.getName() + " is of type (" + fieldToInit.getType() +") which the current auto config shuffle baord code cannot process, curent avalible types are int, double, float , and boolean.");
        }
    }

    
}